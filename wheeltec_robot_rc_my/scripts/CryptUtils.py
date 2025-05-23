import hashlib
import os
import base64
from Crypto.Cipher import DES
from gmssl.sm4 import CryptSM4, SM4_ENCRYPT, SM4_DECRYPT
from gmssl.func import list_to_bytes

DES_KEY = b"12345678"
DES_MODE = DES.MODE_ECB

SM4_KEY = b'0123456789abcdef'
crypt_sm4 = CryptSM4()

AES_KEY = ""
AES_MODE = ""

RSA_KEY = ""


def DESencrpyt(text, key=DES_KEY, mode=DES_MODE):
    '''DES 加密 PKCS5Padding'''
    num = DES.block_size - len(text) % DES.block_size  # 需要填充的字符个数
    text_pad = (text + num * chr(num)).encode('utf-8')  # 填充后的字节串
    crpytor = DES.new(key, mode)
    encrypt_data = crpytor.encrypt(text_pad)  # 对数据进行加密
    return base64.b64encode(encrypt_data).decode('utf-8')


def DESdecrypt(text, key=DES_KEY, mode=DES_MODE):
    '''DES 解密'''
    data = base64.b64decode(text.encode())
    crpytor = DES.new(key, mode)
    decrypt_data = crpytor.decrypt(data)  # 对数据进行解密
    res = decrypt_data[:-decrypt_data[-1]].decode()  # 去除多余字符
    return res


def SM4_encrypt(text):
    global crypt_sm4
    if type(text) == str:
        text = text.encode('utf-8')
    crypt_sm4.set_key(SM4_KEY, SM4_ENCRYPT)
    ciphertext = crypt_sm4.crypt_ecb(text)
    return base64.b64encode(ciphertext).decode('utf-8')


def SM4_decrypt(text):
    global crypt_sm4
    crypt_sm4.set_key(SM4_KEY, SM4_DECRYPT)
    # decrypted = crypt_sm4.crypt_ecb(list_to_bytes([214, 197, 217, 128, 243, 98, 28, 93, 123, 220, 241, 125, 8, 48, 111, 249, 209, 77, 112, 120, 210, 243, 166, 127, 224, 13, 50, 33, 219, 177, 213, 86]))
    return crypt_sm4.crypt_ecb(list_to_bytes(base64.b64decode(text)))


def AESencrpyt(text, key=AES_KEY, mode=AES_MODE):
    pass


def AESdecrypt(text, key=AES_KEY, mode=AES_MODE):
    pass


def RSAencrpyt(text, key=RSA_KEY):
    pass


def RSAdecrypt(text, key=RSA_KEY):
    pass


if __name__ == '__main__':
    m = "hello11"
    print(DESencrpyt(m))
    print(DESdecrypt(DESencrpyt(m)))
    print("---------------------")
    # 明文数据（必须是16字节的倍数）
    plaintext = 'HelloWorld123456'  # 16字节
    print(SM4_encrypt(plaintext))
