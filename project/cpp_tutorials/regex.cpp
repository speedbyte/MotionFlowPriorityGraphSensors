/**

 metacharacter : |
 character class : [] can match just a single character in the target text.
 alternation class : () each alternative can match an arbitrary number of text. Alternation cannot be negated.
 Bob|Robert - this is called alternatives
 grey|gray is the same as gr(a|e)y : gray or grey not the same as gra|ey : gra or ey
 Metasequences : <> - they are not metacharacters.
 The expression \<cat\> literally means “match if we can find a start-of-word position, followed immediately by
 c·a·t, followed immediately by an end-of-word position. Its similiar to ^ and $ for a line.

 */

int main ( int argc, char *argv[] ) {

}