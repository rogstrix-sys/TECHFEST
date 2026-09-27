# Project Development & Modification Guidelines

## Surgical & Targeted Edits (Strict Rule)
- **Access and modify ONLY the parts that need to change**:
  - Make strictly targeted, localized diffs when fixing bugs, refactoring, or introducing new features.
  - Never rewrite entire files or modify unrelated functions, classes, lines, comments, styles, or configurations.
  - All existing code, architecture, parameters, layouts, and working logic must remain identical and preserved unless explicitly instructed otherwise.
- **Preserve Verified Implementations**:
  - Keep all working features, visual HUD layouts, theater view mode defaults, anti-clipping responsive styles, battery physics models, and test suites completely intact.
