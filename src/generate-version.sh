current_branch=`git symbolic-ref HEAD 2> /dev/null | cut -b 12-`
current_commit=`git rev-parse --short=7 HEAD`
if [ -z "$current_branch" ]; then
  current_branch="2.0.0c"
fi
echo "$current_branch-$current_commit"
touch version.cpp
