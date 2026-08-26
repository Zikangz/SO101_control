# Feetech SDK Install Fix on Ubuntu 20.04

## Symptom

Installing `feetech-servo-sdk` fails with:

```text
AttributeError: module 'importlib_metadata' has no attribute 'EntryPoints'
```

## Cause

This is not a Feetech motor problem. It is a Python packaging mismatch:

```text
setuptools from ~/.local is new
importlib_metadata from /usr/lib/python3/dist-packages is old
```

On this machine, `setuptools` imports the old `importlib_metadata 1.5.0`, then crashes.

## Fix

Run this first:

```bash
python3 -m pip install --user -U \
  "importlib-metadata>=6.8,<8" \
  "setuptools>=65,<70" \
  "wheel>=0.38,<0.46"
```

Then verify:

```bash
python3 - <<'PY'
import setuptools
import importlib_metadata
print("setuptools", setuptools.__version__)
print("importlib_metadata", importlib_metadata.version("importlib-metadata"))
PY
```

Then install the SO101 Noetic hardware dependencies:

```bash
cd $SO101_ROOT
python3 -m pip install --user -r requirements-noetic.txt
```

Finally check:

```bash
python3 - <<'PY'
import scservo_sdk
import serial
print("scservo_sdk ok")
print("pyserial ok")
PY
```

## Do Not

Do not upgrade system Python or replace Ubuntu's `/usr/bin/python3`.

Use only the power supply that matches the actual SO101 follower motor variant; this project follower is now recorded as 12V.
