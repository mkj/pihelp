#!/Users/matt/tmp/crypto/bin/python

from Crypto.Cipher import AES
import hashlib
import hmac
from binascii import hexlify, unhexlify

# echo -n '1234567890123456' | openssl enc -aes-128-ecb -K '61616262636364646565666667676868' | hexdump -C

# output:
#cli_key yyuuiiooddeeffqqddii, hex 7979757569696f6f646465656666717164646969
#enc_key aabbccddeeffgghh, hex 61616262636364646565666667676868
#data    1234567890123456, hex 31323334353637383930313233343536
#data    1234567890123456, hex 31323334353637383930313233343536
#enc     hex 0ba302059bfc650d4491268448102119
#hmac hex ecd858ee07a8e16575723513d2d072a7565865e4


cli_key = 'yyuuiiooddeeffqqddii'
enc_key = 'aabbccddeeffgghh\0\0\0\0'
indata =  '1234567890123456'

print "cli_key %s, hex %s" % (cli_key, hexlify(cli_key))
print "enc_key %s, hex %s" % (enc_key, hexlify(enc_key))
print "data    %s, hex %s" % (indata, hexlify(indata))

print "data    %s, hex %s" % (indata, hexlify(indata))

a = AES.new(enc_key[:16])
enc = a.encrypt(indata)

print "enc     hex %s" % hexlify(enc)

h = hmac.new(enc_key, 'D:' + enc, hashlib.sha1).digest()
print "enc hmac hex %s" % hexlify(h)

h = hmac.new(cli_key, 'H:' + cli_key, hashlib.sha1).digest()
print "hmac test hex %s" % hexlify(h)


