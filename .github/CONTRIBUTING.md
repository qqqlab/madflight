# Welcome to madflight contributing guide

Thank you for investing your time in contributing to _madflight_!

## New Contributor

To get an overview of the project, read the [README](../README.md) file. Here are some resources to help you get started with open source contributions:

- [Finding ways to contribute to open source on GitHub](https://docs.github.com/en/get-started/exploring-projects-on-github/finding-ways-to-contribute-to-open-source-on-github)
- [Set up Git](https://docs.github.com/en/get-started/git-basics/set-up-git)
- [GitHub flow](https://docs.github.com/en/get-started/using-github/github-flow)
- [Collaborating with pull requests](https://docs.github.com/en/github/collaborating-with-pull-requests)

## Issue

Please only create a new Issue if you found a bug in the latest version of _madflight_. For all other issues, questions, suggestions, etc. please create a new [Discussion](https://github.com/qqqlab/madflight/discussions)

## Pull Request

Pull Requests are very welcome. Please keep in mind that _madflight_ supports multiple development environments (PlatformIO and Arduino IDE) and multiple microcontroller families (ESP32, RP2, STM32). Your PR needs to compile on all combinations before it can be merged. To make life easier a CI workflow will run these compile checks for you when you submit a PR.

Coding Guidelines

- Write readable code
- Add comments
- Only use #if and #define as last resort
- Copy external libraries

Copy external libraries: this will prevent _madflight_ from breaking when the external lib changes. Remove all examples and unused files from the lib copy, keep only a README file with the external lib license and links to the external project.
