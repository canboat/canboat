#include <common.h>
#include "pgn.h"
#include <assert.h>

void pgnAreSorted() {
  Pgn* it = pgnListFirst();
  for (++it; it != pgnListEnd(); ++it) {
    assert((it - 1)->pgn <= it->pgn);
  }
}

void searchForPgnReturnsTheFirstPgn() {
  Pgn* it;
  for (it = pgnListFirst(); it != pgnListEnd(); ++it) {
    Pgn *searched = searchForPgn(it->pgn);
    assert(searched->pgn == it->pgn);
    assert((searched->pgn == 0) || ((searched - 1)->pgn < searched->pgn));
  }
}

int main() {
  pgnAreSorted();
  searchForPgnReturnsTheFirstPgn();
  return 0;
}


