import json
import os

class JSONDataManager:
    def __init__(self, file_name="config.json"):
        if os.path.isabs(file_name):
            self.filename = file_name
        else:
            current_dir = os.path.dirname(os.path.abspath(__file__))
            ws_root = os.path.abspath(os.path.join(current_dir, "..", "..", ".."))
            self.filename = os.path.join(ws_root, "data", file_name)
        
        self.ensure_file_exists()

    def ensure_file_exists(self):
        os.makedirs(os.path.dirname(self.filename), exist_ok=True)
        
        if not os.path.isfile(self.filename):
            with open(self.filename, 'w') as f:
                json.dump([], f) 
            print(f"Datenbank neu erstellt unter: {self.filename}")

        def _read_all(self):
            with open(self.filename, 'r') as f:
                return json.load(f)
            
        def _write_all(self, data):
            with open(self.filename, 'w') as f:
                json.dump(data, f, indent=4)

        def create(self, item):
            data = self._read_all()
            data.append(item)
            self._write_all(data)
            return "Item created successfully."
        
        def read(self, key=None, value=None):
            data = self._read_all()
            if key and value:
                return [item for item in data if item.get(key) == value]
            return data
        
        def update(self, search_key, search_value, update_key, update_value):
            data = self._read_all()
            for item in data:
                if item.get(search_key) == search_value:
                    item[update_key] = update_value
            self._write_all(data)
            return "Item(s) updated successfully."
        
        def delete(self, search_key, search_value):
            data = self._read_all()
            data = [item for item in data if item.get(search_key) != search_value]
            self._write_all(data)
            return "Item(s) deleted successfully."
