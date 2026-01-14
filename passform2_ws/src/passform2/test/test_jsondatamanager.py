import pytest
import os
import json
from passform2.JSONDataManager import JSONDataManager 

@pytest.fixture
def temp_db(tmp_path):
    file_path = tmp_path / "test_data.json"
    return JSONDataManager(str(file_path))

def test_create(temp_db):
    res = temp_db.create({"id": 1, "name": "test"})
    assert res == "Item created successfully."
    assert len(temp_db.read()) == 1

def test_read_filter(temp_db):
    temp_db.create({"id": 1, "tag": "A"})
    temp_db.create({"id": 2, "tag": "B"})
    res = temp_db.read("tag", "A")
    assert len(res) == 1
    assert res[0]["id"] == 1

def test_update(temp_db):
    temp_db.create({"id": 1, "status": "off"})
    temp_db.update("id", 1, "status", "on")
    res = temp_db.read("id", 1)
    assert res[0]["status"] == "on"

def test_delete(temp_db):
    temp_db.create({"id": 1})
    temp_db.delete("id", 1)
    assert len(temp_db.read()) == 0

def test_file_initialization(tmp_path):
    f = str(tmp_path / "init.json")
    db = JSONDataManager(f)
    assert os.path.exists(f)
    with open(f, "r") as file:
        assert json.load(file) == []