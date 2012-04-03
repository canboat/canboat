#ifndef __cplusplus /* { */
  typedef enum { false = 0, true = 1 } _Bool;
# define true 1
# define false 0
#else /* } { */
  typedef bool _Bool;
# define true true
# define false false
#endif /* } */
#define bool _Bool
