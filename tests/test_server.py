"""
AEGIS-TACTICAL-ISR: Server API & WebSocket Integration Tests
============================================================
Tests REST endpoints, health telemetry status, and GeoJSON stream formatting.
"""

import pytest
from fastapi.testclient import TestClient
from server.app import app


@pytest.fixture
def client():
    return TestClient(app)


def test_api_health_endpoint(client):
    response = client.get("/api/v1/health")
    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "OPERATIONAL"
    assert data["system"] == "AEGIS-TACTICAL-ISR"
    assert "Army Design Bureau" in data["target_organization"]
    assert "georeferencing_engine" in data


def test_api_specs_endpoint(client):
    response = client.get("/api/v1/specs")
    assert response.status_code == 200
    data = response.json()
    assert "sensors" in data
    assert "gimbal_payload" in data
    assert "rgb_daylight" in data["sensors"]
    assert "thermal_flir" in data["sensors"]


def test_api_tracks_geojson_endpoint(client):
    response = client.get("/api/v1/tracks")
    assert response.status_code == 200
    data = response.json()
    assert data["type"] == "FeatureCollection"
    assert isinstance(data["features"], list)


def test_telemetry_injection_endpoint(client):
    test_packet = {
        "uav_id": "GARUDA-01",
        "lat": 34.1526,
        "lon": 77.5771,
        "alt_msl_m": 4800.0,
        "alt_agl_m": 900.0,
        "groundspeed_kmh": 130.0,
        "heading_deg": 90.0,
        "roll_deg": 0.0,
        "pitch_deg": 0.0,
        "yaw_deg": 90.0,
        "gimbal_roll_deg": 0.0,
        "gimbal_pitch_deg": -45.0,
        "gimbal_yaw_deg": 0.0,
        "sensor_mode": "FLIR",
    }
    response = client.post("/api/v1/telemetry/inject", json=test_packet)
    assert response.status_code == 200
    assert response.json()["status"] == "INJECTED_SUCCESSFULLY"


def test_dashboard_root_serves_html(client):
    response = client.get("/")
    assert response.status_code == 200
    assert "AEGIS" in response.text
    assert "Army Design Bureau" in response.text
