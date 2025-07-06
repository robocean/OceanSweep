<script>
  import { onMount } from "svelte";

  let coordinates = []; // 클릭한 GPS 좌표를 저장하는 배열 (로봇 방문 포인트)
  let currentCoordinates = ""; // 현재 클릭한 좌표
  let markers = []; // 지도에 표시된 마커 배열
  let polylines = []; // 지도에 표시된 선 배열
  let backendMarker = null; // 백엔드에서 받은 GPS 위치를 표시하는 마커

  const API_SAVE_URL = "http://127.0.0.1:8000/robocean/save_coordinates"; // FastAPI 경로 저장 엔드포인트

  const KAKAO_API_KEY = "YOUR_KAKAO_API_KEY";

  let map = null; // 지도 객체

  onMount(() => {
    if (navigator.geolocation) {
      navigator.geolocation.getCurrentPosition(successCallback, errorCallback);
    } else {
      alert("Geolocation is not supported by this browser.");
    }

    function successCallback(position) {
      const latitude = position.coords.latitude;
      const longitude = position.coords.longitude;

      const kakaoMapScript = document.createElement("script");
      kakaoMapScript.src = `//dapi.kakao.com/v2/maps/sdk.js?appkey=${KAKAO_API_KEY}&autoload=false`;
      kakaoMapScript.async = true;

      kakaoMapScript.onload = () => {
        kakao.maps.load(() => {
          const container = document.getElementById("map");
          const options = {
            center: new kakao.maps.LatLng(latitude, longitude),
            level: 1,
          };
          map = new kakao.maps.Map(container, options);

          const centerPosition = map.getCenter();
          const infoWindow = new kakao.maps.InfoWindow({
            position: centerPosition,
            content: '<div style="padding:5px; font-size:14px;">Server</div>',
            removable: false,
          });
          infoWindow.open(map, null);

          // 지도 클릭 이벤트 등록
          kakao.maps.event.addListener(map, "click", function (mouseEvent) {
            addUserPoint(mouseEvent.latLng);
          });

          // WebSocket 연결
          connectWebSocket(map);
        });
      };

      document.head.appendChild(kakaoMapScript);
    }

    function errorCallback(error) {
      alert(`Error: ${error.message}`);
    }
  });

  // 사용자 클릭으로 GPS 포인트 추가
  function addUserPoint(latLng) {
    const lat = latLng.getLat();
    const lng = latLng.getLng();

    currentCoordinates = `현재 찍은 좌표: ${lat}, ${lng}`;
    coordinates = [...coordinates, { lat, lng }];

    // 빨간 점 추가
    const circle = new kakao.maps.Circle({
      center: latLng,
      radius: 1,
      strokeWeight: 1,
      strokeColor: "#FF0000",
      strokeOpacity: 1,
      strokeStyle: "solid",
      fillColor: "#FF0000",
      fillOpacity: 1,
    });
    circle.setMap(map);
    markers.push(circle);

    // 점 클릭 이벤트로 삭제 기능 추가
    kakao.maps.event.addListener(circle, "click", () => {
      removeUserPoint(latLng, circle);
    });
  }

  // GPS 포인트 삭제
  function removeUserPoint(latLng, marker) {
    const lat = latLng.getLat();
    const lng = latLng.getLng();

    // 배열에서 좌표 제거
    coordinates = coordinates.filter(
      (coord) => coord.lat !== lat || coord.lng !== lng
    );

    // 지도에서 마커 제거
    marker.setMap(null);
    markers = markers.filter((m) => m !== marker);
  }

    function updateMapWithGPS(gpsData) {
    const position = new kakao.maps.LatLng(gpsData.lat, gpsData.lng);

    // 기존 마커 제거
    if (backendMarker) {
        backendMarker.setMap(null);
    }

    // 새로운 마커 추가
    backendMarker = new kakao.maps.Circle({
        center: position,
        radius: 2,
        strokeWeight: 1,
        strokeColor: "#0000FF", // 파란색
        strokeOpacity: 1,
        strokeStyle: "solid",
        fillColor: "#0000FF",
        fillOpacity: 1,
    });
    backendMarker.setMap(map);
  }

  async function savePath() {
    if (coordinates.length === 0) {
      alert("저장할 좌표가 없습니다!");
      return;
    }

    try {
      const response = await fetch(API_SAVE_URL, {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({ coordinates }),
      });

      if (response.ok) {
        alert("경로가 성공적으로 저장되었습니다!");
      } else {
        const error = await response.json();
        alert(`서버에서 오류가 발생했습니다: ${error.detail}`);
      }
    } catch (error) {
      alert(`서버와의 연결에 실패했습니다: ${error.message}`);
    }
  }

  function resetMap() {
    coordinates = [];
    currentCoordinates = "";

    // 지도에서 모든 마커 제거
    markers.forEach((marker) => marker.setMap(null));
    markers = [];

    // 지도에서 모든 선 제거
    polylines.forEach((polyline) => polyline.setMap(null));
    polylines = [];

    // 백엔드 마커 제거
    if (backendMarker) {
      backendMarker.setMap(null);
      backendMarker = null;
    }
  }
</script>

<div class="container">
  <div class="map-container">
    <div id="map" class="map"></div>
    <p class="current-coordinates">{currentCoordinates}</p>
  </div>

  <div class="sidebar">
    <button on:click={savePath} class="save-button">경로 생성하기</button>
    <button on:click={resetMap} class="reset-button">초기화</button>
    <ul>
      {#each coordinates as coord, index}
        <li>{index + 1}번 포인트: {coord.lat}, {coord.lng}</li>
      {/each}
    </ul>
  </div>
</div>

<style>
  .container {
    display: flex;
    flex-direction: row;
    gap: 20px;
  }

  .map-container {
    display: flex;
    flex-direction: column;
    width: 900px;
  }

  .map {
    height: 600px;
    border: 1px solid #ddd;
  }

  .current-coordinates {
    margin-top: 10px;
    font-size: 16px;
    color: #000;
    font-weight: bold;
  }

  .sidebar {
    width: 300px;
    display: flex;
    flex-direction: column;
    gap: 10px;
  }

  ul {
    font-size: 14px;
    color: #333;
    list-style: none;
    padding: 0;
  }

  li {
    margin-bottom: 5px;
  }

  button {
    padding: 10px 20px;
    border: none;
    border-radius: 5px;
    cursor: pointer;
  }

  .save-button {
    background-color: #4CAF50;
    color: white;
  }

  .reset-button {
    background-color: #007BFF;
    color: white;
  }

  button:hover {
    opacity: 0.9;
  }
</style>
