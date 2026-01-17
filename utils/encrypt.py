from cryptography.fernet import Fernet

# 1. Schlüssel generieren (Nur einmal machen!)
# Speichere diesen Schlüssel in einer Datei, die NICHT auf GitHub landet
def generate_key():
    key = Fernet.generate_key()
    with open("secret.key", "wb") as key_file:
        key_file.write(key)
    print("[+] Schlüssel 'secret.key' wurde erstellt. HALTE IHN GEHEIM!")

# 2. String verschlüsseln
def encrypt_string(plain_text):
    with open("secret.key", "rb") as key_file:
        key = key_file.read()
    f = Fernet(key)
    encrypted = f.encrypt(plain_text.encode())
    print(f"Original: {plain_text}")
    print(f"Verschlüsselt: {encrypted.decode()}")

if __name__ == "__main__":
    # generate_key() # Nur beim ersten Mal entkommentieren!
    encrypt_string("DEIN_WLAN_PASSWORT")