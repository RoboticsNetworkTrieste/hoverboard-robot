# Hoverboard mainboard firmware

The firmware is [hoverboard-firmware-hack-FOC](https://github.com/EFeru/hoverboard-firmware-hack-FOC),
included as a git submodule in `hoverboard-firmware-hack-FOC/`.

Main settings:
- `VARIANT_USART`, control and feedback on USART3 (right sensor cable)
- `CTRL_MOD_REQ SPD_MODE`, `TANK_STEERING`, `ENABLE_ODOMETRY`
- `N_MOT_MAX 200` rpm, `INACTIVITY_TIMEOUT 30` min

PC prerequisites, ST-Link wiring and board unlock: see the [main README](../README.md).

## How the configuration patch works
The submodule always points to an unmodified upstream commit. Our settings are not committed
inside it: they are stored in this folder as `config.patch`, a plain `git diff` of the
submodule. It touches two files:

| File | Changes |
|---|---|
| `Inc/config.h` | Control variant, control mode, limits, timeouts |
| `platformio.ini` | `default_envs = VARIANT_USART` |

This keeps our changes small, readable in one file, and easy to carry over to a newer upstream version.

`build.sh` uses the patch as follows:
1. Stops if the submodule has local changes, so it never mixes the patch with stray edits.
2. Applies `config.patch` to the submodule.
3. Runs `pio run` with your arguments.
4. On exit (success, error or Ctrl+C), reverts the patch, leaving the submodule clean.

While a build runs, `git status` in this repo shows the submodule as `modified content`. That is expected.

## Build and flash
```
git submodule update --init      # first time only
./build.sh                       # build only
./build.sh -t upload             # build and flash through the ST-Link
```
Output: `hoverboard-firmware-hack-FOC/.pio/build/VARIANT_USART/firmware.bin`.

## Changing the configuration
Apply the patch by hand, edit, then save the new diff and clean the submodule:
```
cd hoverboard-firmware-hack-FOC
git apply ../config.patch          # start from the current configuration
# edit Inc/config.h, platformio.ini
git diff > ../config.patch         # save the full set of changes
git checkout .                     # restore the submodule
cd ..
./build.sh                         # check that it still builds
```
`git diff` always captures all changes with respect to upstream, so the new `config.patch`
replaces the old one entirely. Commit `config.patch` in this repo, never inside the submodule.

## Updating the upstream firmware
```
cd hoverboard-firmware-hack-FOC
git fetch origin
git checkout <commit or origin/main>
git apply --reject ../config.patch
```
If some hunks fail, git writes them to `*.rej` files next to the affected files: apply those
changes by hand, then delete the `.rej` files. Finally:
```
git diff > ../config.patch
git checkout .
cd ..
./build.sh
git add hoverboard-firmware-hack-FOC config.patch   # new submodule commit + updated patch
```

## Troubleshooting
`Submodule has local changes, commit them to config.patch or discard them first.`
The submodule was left modified, e.g. a build was killed before it could revert the patch, or
files were edited by hand. Check with `git -C hoverboard-firmware-hack-FOC diff`, then either save the
changes into `config.patch` as described above, or discard them:
```
git -C hoverboard-firmware-hack-FOC checkout .
```
