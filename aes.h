#ifndef AES_H
#define AES_H

// 4*nB*(nK+1)
#define AES_EXPKEY_SIZE (4*4*(4+1))

void ExpandKey (unsigned char *key, unsigned char *expkey);
// encrypt one 128 bit block
void Encrypt (unsigned char *in, unsigned char *expkey, unsigned char *out);

void Decrypt (unsigned char *in, unsigned char *expkey, unsigned char *out);




#endif