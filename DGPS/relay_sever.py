import socket
import threading

# 서버 설정
RELAY_HOST = '0.0.0.0'  # 모든 인터페이스에서 접속 허용
RELAY_PORT = YOUR_PORT_NUM  # 포트 번호

# 클라이언트 목록 (addr -> socket)
clients = {}

# 클라이언트 처리 함수
def handle_client(client_socket, addr):
    print(f"[연결됨] {addr} 클라이언트 연결됨")
    
    try:
        while True:
            data = client_socket.recv(1024)  # 데이터 수신
            if not data:
                break
            print(f"[수신] {addr}: {data.decode()}")
            
            # 데이터를 모든 다른 클라이언트에게 전송
            for client_addr, sock in clients.items():
                if client_addr != addr:  # 자신을 제외하고 전송
                    try:
                        sock.sendall(data)
                    except Exception as e:
                        print(f"[오류] {client_addr}에게 데이터 전송 실패: {e}")
    except Exception as e:
        print(f"[오류] {addr}: {e}")
    finally:
        print(f"[연결 종료] {addr} 클라이언트 연결 해제")
        del clients[addr]
        client_socket.close()

# 서버 실행 함수
def start_relay_server():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind((RELAY_HOST, RELAY_PORT))
    server.listen(5)
    print(f"[서버 시작] {RELAY_HOST}:{RELAY_PORT} 에서 대기 중...")
    
    while True:
        client_socket, addr = server.accept()
        clients[addr] = client_socket
        print(f"[새 연결] {addr} 연결됨")
        
        client_thread = threading.Thread(target=handle_client, args=(client_socket, addr))
        client_thread.start()

if __name__ == "__main__":
    start_relay_server()
