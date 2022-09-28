import json

def read_JSON(file):
        try:
            with open(file, "r") as f:
                data = json.load(f)
        except IOError:
            data = {}

        return data

def save_JSON(file, obj):
    with open(file, 'w') as f:
        json.dump(obj, f)