from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, RedirectResponse
from typing import List
from pyproj import CRS, Transformer
import os
import json

app = FastAPI()

# CORS 설정
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # 모든 도메인 허용 (보안을 위해 필요한 도메인만 명시하는 것이 좋습니다)
    allow_credentials=True,
    allow_methods=["*"],  # 모든 HTTP 메서드 허용
    allow_headers=["*"],  # 모든 헤더 허용
)

# GPS 데이터 모델
class SingleGPS(BaseModel):
    lat: float
    lng: float

class GPSData(BaseModel):
    coordinates: List[SingleGPS]  # {"lat": float, "lng": float} 형태의 배열

# TM 변환기 정의
crs_wgs84 = CRS.from_epsg(4326)
crs_tm = CRS.from_epsg(5175)
transformer = Transformer.from_crs(crs_wgs84, crs_tm, always_xy=True)

# JSON 파일 저장 경로
SAVE_DIR = r"C:\ws\ws_fastapi\test_api\json"
os.makedirs(SAVE_DIR, exist_ok=True)  # 저장 경로가 없으면 생성
JSON_FILE = os.path.join(SAVE_DIR, "gps_info.json")

@app.get("/")
async def root():
    """
    기본 경로를 /robocean으로 리디렉션합니다.
    """
    return RedirectResponse(url="/robocean")


@app.get("/robocean")
async def home():
    """
    기본 페이지로 안내 메시지를 반환합니다.
    """
    return {"message": "Welcome to Robocean API. Available endpoints: /save_coordinates, /get_coordinates, /send_gps"}


@app.post("/robocean/save_coordinates")
async def save_coordinates(data: GPSData):
    """
    프론트엔드로부터 GPS 데이터를 받아 JSON 파일로 저장합니다.
    """
    try:
        tm_data = {"coordinates": []}

        for coord in data.coordinates:
            # 위경도 → TM 좌표 변환
            tm_x, tm_y = transformer.transform(coord.lng, coord.lat)

            # 변환된 좌표 저장
            tm_data["coordinates"].append({
                "x": tm_x,
                "y": tm_y
            })

        # TM 좌표를 JSON 파일로 저장
        with open(JSON_FILE, "w") as file:
            json.dump(tm_data, file, indent=4)

        return {"message": "TM 좌표로 변환되어 저장되었습니다.", "file_path": JSON_FILE}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"TM 변환 또는 저장 중 오류: {e}")

@app.get("/robocean/get_coordinates")
async def get_coordinates():
    """
    저장된 JSON 파일을 클라이언트에게 반환합니다.
    """
    try:
        # JSON 파일이 존재하는지 확인
        if not os.path.exists(JSON_FILE):
            raise HTTPException(status_code=404, detail="저장된 JSON 파일이 없습니다.")

        # JSON 파일 반환
        return FileResponse(JSON_FILE, media_type="application/json", filename="gps_info.json")
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"파일 반환 중 오류가 발생했습니다: {e}")