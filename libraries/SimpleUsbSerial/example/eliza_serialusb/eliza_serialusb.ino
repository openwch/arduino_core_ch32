// This is sample code demonstrating how to use serial over USB
// The Eliza chatbot code is adapted from https://github.com/itmm/eliza

#include <SimpleUsbSerial.h>

//keep it the same as in eliza_chatbot.ino
#define MAX_INPUT_BUFFER_SIZE 80

char input_buffer[MAX_INPUT_BUFFER_SIZE];
char output_buffer[MAX_INPUT_BUFFER_SIZE * 2];
int input_buffer_index = 0;

void setup() {
  // put your setup code here, to run once:
  SerialUSB.begin();
  input_buffer_index = 0;

  while (!SerialUSB) {
    ;  // wait for serial port to connect. Needed for native USB
  }
  SerialUSB.println();
  SerialUSB.println("This is sample code to use serial over USB, using chatbot eliza as example.");
  SerialUSB.println();
  SerialUSB.print("Eliza: ");
  SerialUSB.println(print_generic_response());
}

void loop() {
  // put your main code here, to run repeatedly:
  int incoming = SerialUSB.available();
  if (incoming > 0) {
    char c = SerialUSB.read();
    if (c == '\n' || c == '\r') {
      input_buffer[input_buffer_index] = 0;  //null terminate
      input_buffer_index = 0;
      SerialUSB.print("You: ");
      SerialUSB.println(input_buffer);  //echo back

      if (strnstr(input_buffer, "bye", MAX_INPUT_BUFFER_SIZE) == input_buffer) {
        SerialUSB.print("Eliza: ");
        SerialUSB.println("Your bill will be mailed to you.");
        //break; //exit loop
      } else {
        int output_length = respond(input_buffer, output_buffer);
        if (output_length > 0) {
          SerialUSB.print("Eliza: ");
          SerialUSB.println(output_buffer);
        }
      }
    } else {
      if (input_buffer_index < MAX_INPUT_BUFFER_SIZE - 1) {
        input_buffer[input_buffer_index++] = c;
      }
    }
  }
  SerialUSB.flush();  //if not flushed, data will stay in buffer until buffer is full
}
