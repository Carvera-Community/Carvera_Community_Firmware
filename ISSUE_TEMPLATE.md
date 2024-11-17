## How to file an issue

If you encounter an issue using the Carvera Community Firmware, you are welcome to
[submit an issue](https://github.com/Carvera-Community/Carvera_Community_Firmware/issues)

Since this firmware is a modification to the carvera core firmware, please do not pester Makera with bug reports while using it. 

**YOU MUST** provide a solid summary of the issue you are facing, and any supporting screenshots/pictures. Often an image of the MDI/error screen will go a long way to sorting out a problem. If you can provide steps to replicate the issue including any offending gcodes that cause it, that will also help. 

If you have a feature request, feel free to use the issue tracker to bring it to our attention, or message one of us. 
We try to keep to the LinuxCNC/Faunic style standard as much as possible, so if you can link to any documentation in that sphere it would be helpful.


**DO NOT ASK QUESTIONS HERE** they will not be answered and the issue will be summarily closed.

Before you do that for the first time though please take a moment to read the
following section *completely*. Thank you! :)

### What should I do before submitting an issue?

1. make sure you are running the community firmware and controller and check through the change notes to see if it has already been addressed. 

2. Please make sure to **test out the current edge version** of the firmware and controller to see
   whether the problem you are encountering still exists.

3. The problem still exists? Then please look through the
   [existing tickets](https://github.com/Carvera-Community/Carvera_Community_Firmware/issues?q=is%3Aopen+is%3Aissue)
   to check if there already exists a report of the issue you are encountering.

   **Very important:** Please make absolutely sure that if you find a bug that looks like
   it is the same as your's, it actually behaves the same as your's. E.g. if someone gives steps
   to reproduce his bug that looks like your's, reproduce the bug like that if possible,
   and only add a "me too" if you actually can reproduce the same
   issue. Also **provide all information** as [described below](#what-should-i-include-in-a-bug-report)
   and whatever was additionally requested over the course of the ticket
   even if you "only" add to an existing ticket. The more information available regarding a bug, the higher
   the chances of reproducing and solving it. But "me too" on an actually unrelated ticket
   makes it more difficult due to on top of having to figure out the original problem
   there's now also a [red herring](https://en.wikipedia.org/wiki/Red_herring) interfering - so please be
   very diligent here!

### What should I include in a bug report?

Always use the following template (you can remove what's within `[...]`, that's
only provided here as some additional information for you), **even if only adding a
"me too" to an existing ticket**:

#### What were you doing?

    [Please be as specific as possible here. The maintainers will need to reproduce
    your issue in order to fix it and that is not possible if they don't know
    what you did to get it to happen in the first place. If you encountered
    a problem with specific files of any sorts, make sure to also include a link to a file
    with which to reproduce the problem.]

#### What did you expect to happen?

#### What happened instead?

#### Branch & Commit or Version of The firmware and controller

    [Can be found with the version command. (@version in the controller MDI)]

#### Operating System

#### Any MDI output or screenshots that help explain the issue


Copy-paste this template **completely**. Do not skip any lines!

It might happen that you are asked to provide a more thorough crash log which you can find out how to do [here](http://smoothieware.org/mri-debugging
)
