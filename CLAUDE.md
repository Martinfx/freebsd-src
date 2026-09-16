# Working notes for this tree

## Git

Make one commit per logical change.  A fix, the reformatting that came with
it, and an unrelated cleanup found along the way are three commits, not one,
even when they touch the same file in the same working session.  Push after
each of them.

## Code

Kernel code follows style(9): hard tabs to indent, four spaces to continue a
line, the opening brace of a function definition on a line of its own,
nothing past column 80.

Do not copy from GPL licensed sources.  Register offsets, bit positions and
the clock topology are facts about the hardware and may be taken from the
MT7623N datasheet; table layout, macro names, identifiers and comment text
must not be taken from another implementation.  When a value cannot be
sourced from documentation, say so in a comment rather than guessing.
