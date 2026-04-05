# Copilot Instructions for Carvera Community Firmware

## Git Safety Policy (Mandatory)
- Do not run any git write action unless the user explicitly asks in the current chat.
- Git write actions include: `git add`, `git commit`, `git push`, `git tag`, `git rebase`, `git merge`, `git cherry-pick`, branch create/delete, history edits, and force-push.
- Default to read-only git commands unless asked (for example: `git status`, `git log`, `git diff`).
- Before any approved git write action, state exactly what will be staged/committed/pushed and wait for confirmation if there is ambiguity.
- Never stage a partial subset of unrelated pending changes without explicitly calling that out first.
- After any user-approved git write action, report the exact command run and the result in chat.

## Change and Communication Policy
- Do not perform autonomous repo state changes that were not requested.
- If multiple change sets are present in the working tree, explicitly distinguish:
  - files changed by this task
  - files pre-existing in dirty state
- If the user asks for code changes only, do not perform git actions.

## Repo Skills
Use these repo skills when relevant:
- Build workflow skill: [.github/skills/carvera-build.md](.github/skills/carvera-build.md)
- Kivy log analysis skill: [.github/skills/carvera-kivy-logs.md](.github/skills/carvera-kivy-logs.md)
