/**
  * Disable ARM semihosting (BKPT 0xAB) for on-target debug and cold boot.
  * semihost_disable.s imports __use_no_semihosting; linker uses --no_semihosting.
  */
#include <stdio.h>

void _sys_exit(int return_code)
{
  (void)return_code;
  while (1)
  {
  }
}

void _ttywrch(int ch)
{
  (void)ch;
}

FILE __stdout;
FILE __stderr;

int fputc(int ch, FILE *f)
{
  (void)f;
  return ch;
}
