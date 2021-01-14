# Regular Expressions

**DEF** **Regular Expression, pattern, corpus**

* A regular expression search function will search through the corpus, returning all texts that match the pattern.

Basic patterns: \(note: `//` is used to mark its content as a RE pattern but is not part of the pattern\)

* sequence of digits and Latin letters \(and other special characters except for these listed below\): search for an identical substring
  * `/Buttercup/` search for a substring `Buttercup`
* `[]` specifies a disjunction of characters to match
  * `/[Bb]/` matches `B` or `b`
* `[-]` specifies a range of characters
  * `/[a-d]/` matches `a`, `b`, `c` or `d`.
  * `/[0-9]/` matches an arbitrary digit
  * `/[A-Z]/` matches an arbitrary upper case letter
* `[^]` specifies any character that does _not_ match expressions following `^` symbol
  * `^` must be the first symbol after `[`, otherwise it simply matches a `^`
  * 也就是说在中括号内部的开头放一个 `^` 就可以对要匹配的内容取反
  * `/[^A-Z]/` matches any character that is not an upper case letter
  * `/[e^]/` matches a letter `e` or `^`
* `?` specifies the preceding character to be optional.
  * `/colou?r/` matches `color` or `colour`.
* `*` \(Kleene star\) means "zero or more occurrences"
  * `/a/` matches null, `a`, `aa`, etc.
* `+` \(Kleene plus\) means "one or more occurrences"
  * `/a/` matches `a`, `aa`, etc.
* Anchors anchor RE to particular places in a string
  * 也就是定位用
  * `^` matches the start of a line.
    * `/^The/` matches a `The` at the start of a line
  * `$` matches the end of a line
    * `/.$/` matches a `.` at the end of a line
* `\b` matches a word boundary, while `\B` matches a non-boundary
  * a word is a sequence of digits, underscores or letters

**DEF** **false positive** - incorrectly matched

**DEF** **false negative** - incorrectly missed

**DEF** increasing **precision** - minimizing false positives

**DEF** increasing **recall** - minimizing false negatives

## reference

[\[1\] Speech and Language Processing](https://web.stanford.edu/~jurafsky/slp3/)

