# Template for mrs nodelet


### This repository contains:
- the script that generates and sets up a project, a "starting point" for your nodelet, with predefined basic functionality
- template directory

### What does the script do?
- clones a project to a new repo
- changes project name, class name and nodelet name in all needed files (default src, default header, package.xml, plugins.xml, launchfile, cmake file)
- sets additional packages in cmake
- removes previous `.git` folder
- optionally: sets description, author name and email

### There are short and long ways of usage
short: <br>
`./mrs_create_nodelet_template.sh -s project_name namespace_name class_name path` <br>
long: <br>
`./mrs_create_nodelet_template.sh [options]` <br>
available options: <br>
```[bash]
    -h      --help                  Show help message.
    -pn     --project-name          Project name. example_project by default.
    -pd     --project-description   Project description. Write description as a text in quotes.
    -cp     --cmake-packages        Used for cmake "find_package". default: 'roscpp mrs_lib std_msgs'
    -nn     --namespace-name        Namespace
    -cn     --class-name            Nodelet class name
    -an     --author-name           Author name
    -ae     --author-email          Should be specified for correct compilation!
    -pp     --project-path          Where to create new project. default: ./ 
```
