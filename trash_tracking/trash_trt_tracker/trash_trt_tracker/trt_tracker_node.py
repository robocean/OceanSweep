import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import cv2
import numpy as np
import pycuda.driver as cuda
import pycuda.autoinit
import tensorrt as trt
import time

class_names = ['bolsa de plastico', 'botella', 'lata de aluminio', 'neumatico', 'red de pesca', 'residuos plasticos']

class TRTTracker(Node):
    def __init__(self):
        super().__init__('trt_tracker_node')

        # ROS 2 publishers
        self.publisher_w = self.create_publisher(Float64, 'w_r', 10)
        self.publisher_v = self.create_publisher(Float64, 'v_r', 10)

        # Control params
        self.kp = 0.005
        self.v_forward = 0.2
        self.img_size = 640
        self.conf_threshold = 0.25
        self.target_fps = 10
        self.inference_interval = 5
        self.frame_count = 0

        # TensorRT
        self.TRT_LOGGER = trt.Logger()
        self.engine = self.load_engine('TRT_MODEL_PATH/trash_trace.trt')
        self.exec_context = self.engine.create_execution_context()
        self.inputs, self.outputs, self.bindings, self.stream = self.allocate_buffers(self.engine)

        # Camera
        self.cap = cv2.VideoCapture(self.get_gst_pipeline(), cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            self.get_logger().error("카메라 열기 실패")
            raise RuntimeError("카메라 열기 실패")

        # Run timer
        self.timer = self.create_timer(1.0 / self.target_fps, self.timer_callback)

    def get_gst_pipeline(self):
        return (
            "nvarguscamerasrc ! "
            "video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=30/1 ! "
            "nvvidconv flip-method=2 ! video/x-raw, width=640, height=480, format=BGRx ! "
            "videoconvert ! video/x-raw, format=BGR ! appsink"
        )

    def load_engine(self, path):
        with open(path, "rb") as f:
            engine_bytes = f.read()
        runtime = trt.Runtime(self.TRT_LOGGER)
        return runtime.deserialize_cuda_engine(engine_bytes)

    def allocate_buffers(self, engine):
        inputs, outputs, bindings = [], [], []
        stream = cuda.Stream()
        for binding in engine:
            size = trt.volume(engine.get_binding_shape(binding))
            dtype = trt.nptype(engine.get_binding_dtype(binding))
            host_mem = cuda.pagelocked_empty(size, dtype)
            device_mem = cuda.mem_alloc(host_mem.nbytes)
            bindings.append(int(device_mem))
            if engine.binding_is_input(binding):
                inputs.append((host_mem, device_mem))
            else:
                outputs.append((host_mem, device_mem))
        return inputs, outputs, bindings, stream

    def infer(self, image):
        np.copyto(self.inputs[0][0], image.ravel())
        cuda.memcpy_htod_async(self.inputs[0][1], self.inputs[0][0], self.stream)
        self.exec_context.execute_async_v2(bindings=self.bindings, stream_handle=self.stream.handle)
        cuda.memcpy_dtoh_async(self.outputs[0][0], self.outputs[0][1], self.stream)
        self.stream.synchronize()
        return self.outputs[0][0]

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("프레임 수신 실패")
            return

        old_h, old_w = frame.shape[:2]
        img_center_x = old_w // 2
        self.frame_count += 1

        if self.frame_count % self.inference_interval != 0:
            return

        img_resized = cv2.resize(frame, (self.img_size, self.img_size))
        img_rgb = cv2.cvtColor(img_resized, cv2.COLOR_BGR2RGB)
        img_np = img_rgb.transpose((2, 0, 1)).astype(np.float32) / 255.0
        input_tensor = np.expand_dims(img_np, axis=0)

        output = self.infer(input_tensor)
        detections = output.reshape(-1, 11)

        best_conf = 0
        best_offset = None
        best_label = None
        best_box = None

        for det in detections:
            class_scores = det[5:]
            class_id = np.argmax(class_scores)
            confidence = class_scores[class_id]
            objectness = det[4]
            total_conf = objectness * confidence

            if total_conf > self.conf_threshold and total_conf > best_conf:
                cx, cy, w, h = det[0:4] * np.array([old_w, old_h, old_w, old_h]) / self.img_size
                best_box = (int(cx - w / 2), int(cy - h / 2), int(w), int(h))
                best_offset = int(cx - img_center_x)
                best_label = class_id
                best_conf = total_conf

        if best_box:
            x, y, bw, bh = best_box
            label_text = f"{class_names[best_label]}: {best_conf:.2f}"
            offset_text = f"offset: {best_offset}"
            cv2.rectangle(frame, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
            cv2.putText(frame, label_text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(frame, offset_text, (x, y + bh + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            w_r = self.kp * best_offset * -1
            v_r = 0.1
            self.get_logger().info(f"[{class_names[best_label]}] offset: {best_offset}, w_r: {w_r:.3f}")
        else:
            w_r = 0.0
            v_r = self.v_forward
            self.get_logger().info("탐지 실패, 전진")

        self.publisher_w.publish(Float64(data=w_r))
        self.publisher_v.publish(Float64(data=v_r))

        cv2.imshow("TRT Tracking", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = TRTTracker()
    try:
        rclpy.spin(node)
    finally:
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
