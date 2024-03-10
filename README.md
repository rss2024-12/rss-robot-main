# Team 12: RSS SP'24
General Info for team. As well as notes and ideas for asynchronous work.

## Personal Notes
 - Had to rename old package to allow it to coexist with wall_follower
 - Added param and launch files, but still need testing to see if they run

## Getting started
1. connect everything: router to power and ethernet, bot to both batteries, laptop to router WiFi.
2. Power on the bot: Click (don't hold) on the battery bank power button, then on the boards power button.
3. Connect to the bot's docker: run `ssh racecar@192.168.1.42` in two different terminals. In one, then run `./run_rostorch.sh`. Once the container starts, run `connect` in the other terminal.
4. Start the motors: In the bot's Docker, run `teleop` to begin both joystick and navigation controls.
5. Have fun: run any built ros2 package to get a specific result, then hold **R1** to let it drive on it's own. Or hold **L1** to drive with the controller.

## Common commands
### File management
 - `scp [copy_target] [copy_location]` copies a file from `[copy_target]` to `[copy_location]`. If either of them are remote (ex: on the robot), set the remote as `[user]@[device_addr]:[location]`.
 - `mkdir [folder_name]` makes a folder.
 - `rm [file_name]` removes a file, if targeting a non-empty folder user the `-r` option.
 - `mv [file_location] [target_location]` moves a file or folder. You can also rename a file by moving it to the same folder, but with a different name.
 - `cp [copy_target] [copy_location]` copies a file, but only localy.


### git
#### commits
Commits keep track of your progress. Everytime you finish a function/feature, create a commit (only if it works, don't push broken code).
- `git status` checks the status of all files. Will say if a file has been deleted, changed, or added since the last commit in the current branch.
- `git add [directory]` stages all files in `[directory]`. If you specify a folder, it stages all files within the folder.

- `git restore --staged [file]` removes file from stage.

- `git commit -m "[message]"` creates a commit with all staged files. Also adds a commit message `[message]`. Use the message to give a 1 line summary of what you've done.

- `git pull` gets all commit in GitHub for the current branch.

- `git push` pushes all local commit in the current branch to GitHub. If the two have different commit histories, you will get a conflict error.

- `git log` checks commit history. Add `--oneline --graph` to make it easier to understand. Add `--all` for everyones branches. as well

#### branching
Keeps your code from iterfering or destroying others code. You work with a specific version of the code until you're done. Then you can merge that into `master` to combine your code for the robot

- `git branch [branch]` creates a new branch.

- `git checkout [branch]` switches to `[branch]`. All current changes have to be commited or stashed.

- `git merge [branch]` tries to combine all changes from `[branch]` to the current branch. If the changes cannot be resolved, it will raise a merge error.

- `git stash` saves all changes to files, so you can change branches with no issues.

- `git stash pop` reapplies the last set of stashed changes

#### others

- `git reset --hard` **ONLY IN EMERGENCIES!!!** resets all files to last commit on branch, with no chance to recover.