using System;

// This used to sit at the top of Algorithms.cs, one of the vendored Wintellect.PowerCollections files.
// When those were deleted the attribute would have been lost silently: nothing fails to build without it,
// but the assembly stops declaring CLS compliance, and the [CLSCompliant(false)] markers scattered over
// OpenGL.cs and DebuggerContainer.cs only make sense against an assembly that claims to be compliant.
[assembly: CLSCompliant(true)]
