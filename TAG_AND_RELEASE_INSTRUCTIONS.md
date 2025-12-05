# Instructions for Creating PG2 Tag and Release

## Background
This repository has a git tag `PG2` created locally, but it cannot be pushed directly due to repository rules that restrict tag creation. The following instructions explain how to create the tag and release in GitHub.

## Option 1: Via GitHub UI (Recommended)

1. Navigate to the repository: https://github.com/ProyectosCE/CE1110_PG2
2. Click on "Releases" (or go to https://github.com/ProyectosCE/CE1110_PG2/releases)
3. Click "Create a new release"
4. In "Choose a tag" dropdown, type `PG2` and click "Create new tag: PG2 on publish"
5. Set "Target" to the branch or commit hash: `4b8cc23` (or `copilot/create-tag-and-release-pg2` branch)
6. Set "Release title" to: `PG2`
7. Set "Description" to:
   ```
   # Release PG2 - Proyecto Grupal 2

   ## Description
   This release marks the completion of Proyecto Grupal 2 (Group Project 2) for the course CE1110 - Análisis de Señales Mixtas (Mixed Signals Analysis).

   ## Project Details
   - **Project:** Diseño de un sistema de control: Control de velocidad de puerta corrediza
   - **Course:** CE1110 - Análisis de Señales Mixtas
   - **Institution:** Instituto Tecnológico de Costa Rica

   ## Students
   - José Bernardo Barquero Bonilla (2023150476)
   - Jimmy Feng Feng (2023060347)
   - Alexander Montero Vargas (2023166058)

   ## Professor
   - Luis Alberto Chavarría Zamora <lachavarria@itcr.ac.cr>
   ```
8. Click "Publish release"

## Option 2: Via GitHub CLI (Requires Admin Access)

If you have admin access and the GitHub CLI installed:

```bash
# Authenticate
gh auth login

# Create release (this will also create the tag)
gh release create PG2 \
  --title "PG2" \
  --notes "# Release PG2 - Proyecto Grupal 2

## Description
This release marks the completion of Proyecto Grupal 2 (Group Project 2) for the course CE1110 - Análisis de Señales Mixtas (Mixed Signals Analysis).

## Project Details
- **Project:** Diseño de un sistema de control: Control de velocidad de puerta corrediza
- **Course:** CE1110 - Análisis de Señales Mixtas
- **Institution:** Instituto Tecnológico de Costa Rica

## Students
- José Bernardo Barquero Bonilla (2023150476)
- Jimmy Feng Feng (2023060347)
- Alexander Montero Vargas (2023166058)

## Professor
- Luis Alberto Chavarría Zamora <lachavarria@itcr.ac.cr>" \
  --target copilot/create-tag-and-release-pg2
```

## Option 3: Adjust Repository Rules

If you want to allow tag creation via git push:

1. Go to https://github.com/ProyectosCE/CE1110_PG2/settings/rules
2. Find the rule that restricts tag creation
3. Either disable it or add an exception for specific users/teams
4. Then you can push the tag using:
   ```bash
   git push origin PG2
   ```

## Files Included

This PR includes:
- `VERSION` - Contains the version identifier "PG2"
- `RELEASE_NOTES.md` - Detailed release notes for PG2
- `TAG_AND_RELEASE_INSTRUCTIONS.md` - These instructions

## Repository Rule Violation

When attempting to push the tag, the following error was encountered:

```
remote: error: GH013: Repository rule violations found for refs/tags/PG2.        
remote: - Cannot create ref due to creations being restricted.
```

This indicates that the repository has rules preventing tag creation via push, which is why manual creation via the GitHub UI or by an administrator is required.
