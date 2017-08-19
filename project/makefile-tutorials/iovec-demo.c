#include <sys/uio.h>
#include <string.h>
#include <stdio.h>

int main ( int argc, char **argv )
{
   char *str0 = "hello ";
   char *str1 = "world\n";
   struct iovec iov[2];
   ssize_t nwritten;

   iov[0].iov_base = str0;
   iov[0].iov_len = strlen(str0);
   iov[1].iov_base = str1;
   iov[1].iov_len = strlen(str1);

   nwritten = writev(2, iov, 2);
}
