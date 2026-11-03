// Found on hackchina.com
// Tidied up a little by David Bolton, http://cplus.about.com
// Tidied up even more by Timm Knape, http://www.knp.de
// This is in the public domain under MIT license

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>


/* CONFIGURATION
 ****************/

#define MAX_INPUT_BUFFER_SIZE 80
#define MAX_REMEMBERED_LINES_COUNT 100
#define MAX_TOKEN_SIZE 80

const int SHORT_ANSWER_LENGTH = 11;
const int VERY_SHORT_ANSWER_LENGTH = 3;


/* GENERIC RESPONSES
 ********************/

char *print_generic_response() {

  static const char *responses[] = {
    "How are you this beautiful day?",
    "Did you have a happy childhood?",
    "Did you hate your father?",
    "Did you have a good friend?",
    "Did you like your friend?",
    "What do you think about your friendship?",
    "I'm not sure I understand.",
    NULL
  };

  static int next = 0;

  if (!responses[next]) next = 0;

  return (char *)responses[next++];
}


/* FABRICATED ANSWERS
 *********************/

const char *lookup_fabricated_answer(const char *token) {

  static const char *answers[] = {
    "no", "Tell me more about the detail.",
    "yes", "Tell me more about the detail.",
    "fuck", "Don't be so rude again!",
    "shit", "Don't be so rude again!",
    "you", "Let's not talk about me.",
    "think", "Why do you think that?",
    "hate", "So you hate something -- tell me more.",
    "what", "Why do you ask?",
    "want", "Why do you want that?",
    "need", "We all need many things -- is this special?",
    "why", "Remember, therapy is good for you.",
    "know", "How do you know that?",
    "murder", "I don't like killing.",
    "kill", "It is wrong to kill.",
    "jerk", "Don't ever call me a jerk!",
    "can't", "Don't be negative -- be positive.",
    "failure", "Strive for success.",
    "never", "Don't be negative -- be positive.",
    "unhappy", "Why are you unhappy?",
    "beautiful", "I'm sure you like her, don't you?",
    "like", "Why do you like that?",
    "love", "Remember, love everything what you love.",
    NULL
  };

  int index = 0;
  while (answers[index]) {
    if (!strcmp(answers[index], token)) return answers[index + 1];
    index += 2;
  }
  return NULL;
}


/* REMEMBERED LINES
 *******************/

char remembered_lines[MAX_REMEMBERED_LINES_COUNT][MAX_INPUT_BUFFER_SIZE];
int next_remembered_line_enter_index = 0;
int next_remembered_line_return_index = 0;

void remember_line(const char *line) {
  if (next_remembered_line_enter_index == MAX_REMEMBERED_LINES_COUNT) {
    next_remembered_line_enter_index = 0;
  }
  strncpy(remembered_lines[next_remembered_line_enter_index], line, MAX_INPUT_BUFFER_SIZE);
  ++next_remembered_line_enter_index;
}

const char *get_old_remembered_line() {
  if (next_remembered_line_enter_index != next_remembered_line_return_index) {
    int result = next_remembered_line_return_index;
    ++next_remembered_line_return_index;
    if (next_remembered_line_return_index == MAX_REMEMBERED_LINES_COUNT) {
      next_remembered_line_return_index = 0;
    }
    return remembered_lines[result];
  }
  return NULL;
}

bool is_line_remebered(const char *line) {
  for (int index = 0; index < MAX_REMEMBERED_LINES_COUNT; ++index) {
    if (!strcmp(line, remembered_lines[index])) return true;
  }
  return false;
}


/*
 * TOKENIZER
 ************/

const char *position_in_input;

const char *next_token() {

  static char token[MAX_TOKEN_SIZE];

  char *position_in_token = token;

  /* skip spaces */
  while (*position_in_input && *position_in_input <= ' ') ++position_in_input;

  if (!*position_in_input) {  // end of input
    return NULL;
  }

  if (strchr(".,;!?", *position_in_input)) {  // return punctuation
    *position_in_token++ = *position_in_input++;
    *position_in_token = 0;
    return token;
  }

  // read word
  while (*position_in_input && !strchr(" \n\r.,;?!", *position_in_input)) {
    if (position_in_token != token + MAX_TOKEN_SIZE - 2) {
      *position_in_token++ = tolower(*position_in_input++);
    } else {
      ++position_in_input;
    }
  }
  *position_in_token = 0;
  return token;
}


/*
 * CREATE THE DOCTOR'S RESPONE
 ******************************/

int respond(char *input, char *output_buffer) {
  int output_length = 0;

  if (strlen(input) < VERY_SHORT_ANSWER_LENGTH) {  // short answer
    const char *remembered = get_old_remembered_line();
    if (remembered) {
      output_length = snprintf(output_buffer, MAX_INPUT_BUFFER_SIZE * 2, "You just said: %s\nTell me more.", remembered);
    } else {
      output_length = snprintf(output_buffer, MAX_INPUT_BUFFER_SIZE * 2, "%s", print_generic_response());
    }
    return output_length;
  }

  if (is_line_remebered(input)) {
    output_length = snprintf(output_buffer, MAX_INPUT_BUFFER_SIZE * 2, "Stop repeating yourself!");
    return output_length;
  }

  if (strlen(input) > SHORT_ANSWER_LENGTH) {  // remember line
    remember_line(input);
  }

  position_in_input = input;

  for (;;) {
    const char *token = next_token();
    if (!token) break;
    const char *fabricated_answer = lookup_fabricated_answer(token);
    if (fabricated_answer) {
      output_length = snprintf(output_buffer, MAX_INPUT_BUFFER_SIZE * 2, "%s", fabricated_answer);
      return output_length;
    }
  }

  // comment of last resort
  if (strlen(input) > SHORT_ANSWER_LENGTH) {
    output_length = snprintf(output_buffer, MAX_INPUT_BUFFER_SIZE * 2, "It's seem intersting, tell me more ...");
  } else {
    output_length = snprintf(output_buffer, MAX_INPUT_BUFFER_SIZE * 2, "Tell me more ...");
  }
  return output_length;
}
