
#include "debug.h"
#include "olsr.h"
#include "scheduler.h"
#include <string.h>

/**
 * "Buffered printf", a simple way of
 * keeping track of strings to print
 * with olsr_process_changes. */

struct buffered_string {
  char *s;
  uint8_t n;
  struct buffered_string *next;
};

static struct buffered_string hook_buffered_strings = { NULL, NUMBER_OF_PRINTS, NULL };

void
olsr_buffered_memdump(const char* name, size_t size, const void* ptr, size_t len)
{
  int i;
  const unsigned char* buff = ptr;
  char* str = olsr_malloc(size + 3*len + 3, "buffered memdump");
  if(str == NULL)
    return;
  memset(str, 0, size + 3*len + 3);
  memcpy(str, name, size);
  str[size] = ':';
  str[size+1] = ':';
  str[size+2] = ' ';
  for(i = 0 ; i < (int) len ; i++)
    sprintf(&str[size + 3 + 3*i], " %02x", buff[i]);
  olsr_buffered_print(str, size + 3*len + 3);
  free(str);
}

void
olsr_buffered_printf(size_t len, const char *format, ...) {
  va_list arglist;
  char *buf = olsr_malloc(len+1, "buffered printf");
  if(buf == NULL)
    return;
  va_start(arglist, format);
  vsnprintf(buf, len+1, format, arglist);
  va_end(arglist);
  olsr_buffered_print(buf, strlen(buf));
  free(buf);
}

void
olsr_buffered_print(const char *s, size_t len) {
  struct buffered_string *b = &hook_buffered_strings;  
  if(len == 0)
    len = strlen(s);
  while(b->s != NULL) {
    if(b->next == NULL) {
      b->next = olsr_malloc(sizeof(struct buffered_string), "buffered print");
      if(b->next == NULL)
        return;
      b->next->s = NULL;
      b->next->n = NUMBER_OF_PRINTS;
      b->next->next = NULL;
    }
    b = b->next;
  }
  b->s = olsr_malloc(len + 1, "buffered print");
  if(b->s != NULL) {
    memcpy(b->s, s, len);
    b->s[len] = '\0';
  }
}

void 
olsr_print_buffered_strings(void)
{
  struct buffered_string *b, *n;
  if(hook_buffered_strings.s == NULL && hook_buffered_strings.next == NULL)
    return;
  OLSR_PRINTF(1, "\n--- %s ------------------------VARIOUS DEBUG BUFFERED STRINGS\n", olsr_wallclock_string());
  if(hook_buffered_strings.s != NULL) {
    OLSR_PRINTF(1, "[%d] %s\n", hook_buffered_strings.n, hook_buffered_strings.s);
    if(--hook_buffered_strings.n == 0) {
      free(hook_buffered_strings.s);
      hook_buffered_strings.s = NULL;
      hook_buffered_strings.n = NUMBER_OF_PRINTS;
    }
  }
  b = &hook_buffered_strings;
  while(b->next != NULL) {
    if(b->next->s != NULL) {
      OLSR_PRINTF(1, "[%d] %s\n", b->next->n, b->next->s);
      if(--b->next->n == 0) {
        free(b->next->s);
        b->next->s = NULL;
      }
    }
    if(b->next->s == NULL) {
      n = b->next->next;
      free(b->next);
      b->next = n;
    } else
      b = b->next;
  }
  return;
}
