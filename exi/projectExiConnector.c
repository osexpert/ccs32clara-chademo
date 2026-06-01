
#include "projectExiConnector.h"
#include <string.h> /* for memcpy */


static uint8_t exiTransmitBuffer[EXI_TRANSMIT_BUFFER_SIZE];
struct dinEXIDocument dinDocEnc;
struct dinEXIDocument dinDocDec;
struct appHandEXIDocument aphsDoc;
bitstream_t global_streamEnc;
bitstream_t global_streamDec;
size_t global_streamEncPos;
size_t global_streamDecPos;
int g_errn;
uint8_t sessionId[SESSIONID_LEN];
uint8_t sessionIdLen;

void projectExiConnector_decode_appHandExiDocument(void) {
   /* precondition: The global_streamDec.size and global_streamDec.data have been set to the byte array with EXI data. */

   global_streamDec.pos = &global_streamDecPos;
   *(global_streamDec.pos) = 0; /* the decoder shall start at the byte 0 */
   g_errn = decode_appHandExiDocument(&global_streamDec, &aphsDoc);
}

void projectExiConnector_decode_DinExiDocument(void) {
   /* precondition: The global_streamDec.size and global_streamDec.data have been set to the byte array with EXI data. */

   global_streamDec.pos = &global_streamDecPos;
   *(global_streamDec.pos) = 0; /* the decoder shall start at the byte 0 */
   g_errn = decode_dinExiDocument(&global_streamDec, &dinDocDec);
}

void projectExiConnector_prepare_DinExiDocument(void) {
   /* before filling and encoding the dinDocEnc, we initialize here all its content. */
   init_dinEXIDocument(&dinDocEnc);
   dinDocEnc.V2G_Message_isUsed = 1u;
   init_dinMessageHeaderType(&dinDocEnc.V2G_Message.Header);
   init_dinBodyType(&dinDocEnc.V2G_Message.Body);
   /* take the sessionID from the global variable: */
   memcpy(dinDocEnc.V2G_Message.Header.SessionID.bytes, sessionId, SESSIONID_LEN);
   dinDocEnc.V2G_Message.Header.SessionID.bytesLen = sessionIdLen;
}

void projectExiConnector_encode_DinExiDocument(void) {
   /* precondition: dinDocEnc structure is filled. Output: global_stream.data and global_stream.pos. */
   global_streamEnc.size = EXI_TRANSMIT_BUFFER_SIZE;
   global_streamEnc.data = exiTransmitBuffer;
   global_streamEnc.pos = &global_streamEncPos;
   *(global_streamEnc.pos) = 0; /* start adding data at position 0 */
   g_errn = encode_dinExiDocument(&global_streamEnc, &dinDocEnc);

}
