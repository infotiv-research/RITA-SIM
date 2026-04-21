# Contributing

Legal Notice: When contributing to this project, you must agree that you have authored 100% of the content, that you have the necessary rights to the content, and that the content you contribute may be provided under the project license.

We try to use:

- [Docker, vscode + dev-container extension](https://code.visualstudio.com/docs/devcontainers/containers) to develop inside a container
- [Markdown](https://www.markdownguide.org/tools/vscode/) for documentation
- [Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/) for commit messages
- [pre-commit](https://pre-commit.com/) for git pre-commit hooks and basic QA
- [Semantic Versioning](https://semver.org/) for versioning
- [Draw.io](https://github.com/jgraph/drawio-desktop) for drawing diagrams for documentation

## Technical

- Avoid optimization unless you have a clear, measurable performance problem, as premature optimization can lead to overly complex and unreadable code.

- [PEP 20 – The Zen of Python](https://peps.python.org/pep-0020/)

  - Simple is better than complex.
  - Readability counts.
  - If the implementation is hard to explain, it's a bad idea.

- Please be generous in giving credit when using an image or a piece of code created by others. Add a link to the original author in [credits.md](credits.md).

- Avoid pushing binary files such as images, models, etc. (use .gitignore), especially if they are constantly changing.

### git

- Try to avoid breaking the `main` branch, but don't be obsessed with having a perfect merge request. We can always revert back to the working version or fix the issues. That is why we are using `git`.
- Follow this simple git process:

```
git checkout main
git pull
git checkout -b "branch_name"
# update files
git commit -m "conventional_commit_type: conventional_commit_message"
git push
# Create a pull request.
```

Common commit types:

- feat: Introduces a new feature.
- fix: Patches a bug or issue.
- chore: Routine maintenance or changes that don't affect the app's functionality.
- docs: Documentation changes.

Conventional commit message:

- A conventional commit message uses the imperative mood in the subject line

Example of

- feat(auth): add login functionality
- fix(ui): resolve button color issue

[https://www.conventionalcommits.org](https://www.conventionalcommits.org)

### Misc

- Don't use absolute paths. Your code should run correctly both inside the VS Code Dev Container and independently outside of it. If you need to modify a configuration file at build time, use relative paths from the project root directory (avoid using ".." as it becomes hard to see which scripts are using a specific directory). For ROS2 resources, use `get_package_share_directory(project_name)` to locate package files.
- When adding a new feature, add it to control.sh. Otherwise, `control.sh` should remain unchanged.
- Any configuration that requires modification should be defined in `config.sh`.
- Bash is for automation and has to be short and clear as it is the starting point to understand the project. No unnecessary error handling is needed. 
- Error handling and synchronization should be done by the consumer of the information. (e.g. if a node needs to read a ros message, it is responsible to check if the ros is accessible and it is up and running).
- Bash should preferably not be used for synchronization. Bash is suppose to be sequential.
- Bash can be used for sync between execution of stand alone script. (e.g running 100 test scripts)

### Documentation

Documentation is done in a markdown file. Try to keep our documentation close to your code. Keep the documentation short and clear.

- `#` : Only used once for the title of the project.
- `##` : usually once in each markdown file and usually shares semantics with the markdown filename.
- `###` : Different topics, try to split your documentation into at least 4 topics.
- `####` : subtopics. Try to avoid when you can use simple paragraphs.

Follow this subset of [Markdown](https://www.markdownguide.org/tools/vscode/) tags.
