# Generating the symdefs for GCC from ARMCC symdef:

Make a copy of the file.

In VS Code, search for regular expression `(0x[0-9a-fA-F]+) . (.*)` and replace with `$2 = $1;`.
Remove the first line (`#<SYMDEFS># ARM Linker...`)

Search for `;.*;\n` replace with nothing (Remove commented lines)

Search for `\n\n` replace with `\n` (Remove empty lines)
