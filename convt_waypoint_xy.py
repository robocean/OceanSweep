import json
import os
from pyproj import Proj

# UTM 변환기 설정 (예: 52N 존, WGS84 기준)
utm_proj = Proj(proj="utm", zone=52, ellps="WGS84", south=False)

def convert_coordinates_to_utm(json_file_path, output_file_path):
    # JSON 파일 불러오기
    with open(json_file_path, 'r') as file:
        data = json.load(file)
    
    coordinates = data.get("coordinates", [])
    if not coordinates:
        print("No coordinates found in the file.")
        return

    # 변환된 좌표 저장
    utm_results = []

    print("Converting GPS Coordinates to UTM Coordinates...")
    for point in coordinates:
        lat = point["lat"]
        lng = point["lng"]

        # 위도/경도 → UTM x, y 변환
        x, y = utm_proj(lng, lat)
        utm_results.append({"x": x, "y": y})

        print(f"Lat: {lat}, Lng: {lng} → UTM X: {x:.2f}, UTM Y: {y:.2f}")

    # 출력 디렉토리 확인 및 생성
    output_dir = os.path.dirname(output_file_path)
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # 결과를 새로운 JSON 파일에 저장
    with open(output_file_path, 'w') as output_file:
        json.dump({"utm_coordinates": utm_results}, output_file, indent=4)

    print(f"\nConverted UTM coordinates have been saved to '{output_file_path}'.")

if __name__ == "__main__":
    # JSON 파일 경로 입력
    input_json_file = "/home/ubuntu/get_coordinates"  # 입력 JSON 파일 경로
    output_json_file = "/home/ubuntu/utm_coordinates.json"  # 출력 JSON 파일 경로


    # UTM 좌표 변환 및 저장
    convert_coordinates_to_utm(input_json_file, output_json_file)
