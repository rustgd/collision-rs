### Submission Checklist

Before submitting your pull request to the repository, please make sure you have
done the following things first:

1. You have ensured the pull request is based on a recent version of your
   respective branch.
2. If your pull request adds new methods or functions to the codebase, you have
   written test cases for them.
   * Unit tests are placed at the bottom of the same .rs file in a submodule
     called `tests`. For an example, see the unit tests in the [frustum.rs][st]
     file.
   * Integration tests are placed in a separate .rs file in the `tests`
     subdirectory.
3. You have processed your source code with `cargo fmt`.
4. All of the following commands completed without errors.
   * `cargo build`
   * `cargo test --all`
5. You have granted non-exclusive right to your source code under the [Apache License 2.0][la]

[la]: LICENSE
[st]: tests/frustum.rs

> If you want to be publicly known as an author, feel free to add your name
> and/or GitHub username to the AUTHORS.md file in your pull request.

Once you have submitted your pull request, please wait for a reviewer to give
feedback on it. If no one responds, feel free to @-mention a developer. Once
your code has been reviewed, revised if necessary, and then signed-off by a
developer, it will be merged into the source tree.