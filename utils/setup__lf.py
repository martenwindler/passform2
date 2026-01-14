import os

start_path = os.path.abspath(os.path.join(os.getcwd(), ".."))

for root, dirs, files in os.walk(start_path):
    if ".git" in root or "build" in root or "install" in root:
        continue
        
    for file in files:
        if not file.startswith('.'):
            path = os.path.join(root, file)
            try:
                with open(path, 'rb') as f:
                    content = f.read()
                
                new_content = content.replace(b'\r\n', b'\n')
                
                if new_content != content:
                    with open(path, 'wb') as f:
                        f.write(new_content)
                    print(f"Converted: {path}")
            except Exception:
                pass