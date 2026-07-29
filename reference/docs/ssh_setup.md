# SSH Key Setup — Generating and registering keys

FARMS uses SSH-based cloning for the main repository and all submodules. This page explains how to generate an SSH key pair, register it on GitHub, and configure your system so that both native Git and Docker builds can use it.

---

## Overview

SSH authentication uses a **key pair**: a private key (stays on your machine, never shared) and a public key (uploaded to GitHub). When you run `git clone git@github.com:...`, Git authenticates using your private key.

---

## Step 1: Check for an existing key

Before generating a new key, check whether you already have one:

```bash
ls ~/.ssh
```

If you see `id_ed25519` + `id_ed25519.pub`, or `id_rsa` + `id_rsa.pub`, you already have a key pair. Skip to [Step 3](#step-3-add-the-public-key-to-github).

---

## Step 2: Generate a new SSH key

**ed25519** is the recommended algorithm — smaller keys, faster operations, and widely supported by GitHub.

```bash
ssh-keygen -t ed25519 -C "your_email@example.com"
```

When prompted:
- **File location**: press Enter to accept the default (`~/.ssh/id_ed25519`)
- **Passphrase**: enter a passphrase (recommended) or press Enter for no passphrase

This creates two files:
- `~/.ssh/id_ed25519` — **private key** (never share this)
- `~/.ssh/id_ed25519.pub` — **public key** (this goes to GitHub)

!!! tip
    If you are on an older system that does not support ed25519, use RSA instead:
    ```bash
    ssh-keygen -t rsa -b 4096 -C "your_email@example.com"
    ```
    This creates `~/.ssh/id_rsa` and `~/.ssh/id_rsa.pub`.

---

## Step 3: Add the public key to GitHub

Copy your public key to the clipboard:

```bash
# Linux:
cat ~/.ssh/id_ed25519.pub
# Then select and copy the output manually, or:
cat ~/.ssh/id_ed25519.pub | xclip -selection clipboard

# macOS:
cat ~/.ssh/id_ed25519.pub | pbcopy

# Windows (PowerShell):
Get-Content $env:USERPROFILE\.ssh\id_ed25519.pub | Set-Clipboard
```

Then on GitHub:

1. Go to **GitHub → Settings → SSH and GPG keys** ([direct link](https://github.com/settings/keys))
2. Click **New SSH key**
3. **Title**: any label to identify this machine (e.g. `My Laptop`)
4. **Key type**: Authentication Key
5. **Key**: paste the public key (starts with `ssh-ed25519 AAAA...`)
6. Click **Add SSH key**

---

## Step 4: Test the connection

```bash
ssh -T git@github.com
```

Expected output:
```
Hi <username>! You've successfully authenticated, but GitHub does not provide shell access.
```

If you see `Permission denied (publickey)`, see [Troubleshooting](#troubleshooting) below.

---

## Step 5: Start the SSH agent (required for Docker builds)

The Docker build uses **SSH agent forwarding** to pull FARMS submodules from GitHub during the image build. The agent must be running with your key loaded on the host before running `docker compose up --build`.

**Linux / macOS:**
```bash
eval "$(ssh-agent -s)"
ssh-add ~/.ssh/id_ed25519
```

Verify:
```bash
ssh-add -l
# Should print your key fingerprint
```

**Windows (PowerShell) — one-time setup:**
```powershell
# Run as Administrator:
Set-Service ssh-agent -StartupType Automatic
Start-Service ssh-agent

# Add the key:
ssh-add $env:USERPROFILE\.ssh\id_ed25519
```

To avoid repeating this every session on Linux, add it to your `~/.bashrc` or `~/.zshrc`:
```bash
# Auto-start ssh-agent (add to ~/.bashrc or ~/.zshrc)
if [ -z "$SSH_AUTH_SOCK" ]; then
    eval "$(ssh-agent -s)"
    ssh-add ~/.ssh/id_ed25519 2>/dev/null
fi
```

---

## Step 6: Configure the SSH config file (optional but recommended)

Create or edit `~/.ssh/config` to avoid specifying the key file manually every time:

```
Host github.com
    HostName github.com
    User git
    IdentityFile ~/.ssh/id_ed25519
    AddKeysToAgent yes
```

With `AddKeysToAgent yes`, the key is automatically added to the agent the first time it is used.

---

## Windows: Key file location

On Windows, SSH keys are stored in `%USERPROFILE%\.ssh\` (e.g. `C:\Users\yourname\.ssh\`). The Docker compose file mounts this directory into the container:

```yaml
volumes:
  - ${USERPROFILE}/.ssh:/home/farmsuser/.ssh:ro
```

This makes your host SSH keys available inside the container **read-only**. The `known_hosts` entry for `github.com` is pre-added during the Docker image build (`ssh-keyscan github.com >> ~/.ssh/known_hosts`), so no interactive host verification prompt is shown.

The Windows compose file is configured to use `id_ed25519` by default:
```yaml
ssh:
  - default=${USERPROFILE}/.ssh/id_ed25519
```

If you are using an RSA key, edit `docker_config/windows/docker-compose.yml` and change `id_ed25519` to `id_rsa`.

---

## Troubleshooting

### `Permission denied (publickey)`

1. Verify the agent has your key: `ssh-add -l`
2. Verify the public key is on GitHub: **Settings → SSH keys**
3. Verify the key file permissions:
   ```bash
   chmod 600 ~/.ssh/id_ed25519
   chmod 644 ~/.ssh/id_ed25519.pub
   chmod 700 ~/.ssh
   ```
4. Test with verbose output to see which key is tried:
   ```bash
   ssh -vT git@github.com 2>&1 | grep "Offering\|Authenticated"
   ```

### Multiple GitHub accounts

If you have multiple SSH keys for different GitHub accounts, use the `~/.ssh/config` file to specify which key maps to which host alias:

```
Host github-personal
    HostName github.com
    User git
    IdentityFile ~/.ssh/id_ed25519_personal

Host github-work
    HostName github.com
    User git
    IdentityFile ~/.ssh/id_ed25519_work
```

Then clone using the alias instead of `github.com`:
```bash
git clone git@github-personal:farmsim/farms_zbot.git
```

### Docker build: `Could not resolve host: github.com`

The SSH agent is not running or has no keys loaded. Run:
```bash
ssh-add -l   # must show at least one key
```
If it shows `The agent has no identities`, run `ssh-add ~/.ssh/id_ed25519` and retry the build.

---

## See Also

- [Installation](installation.md) — full environment setup including Docker and native methods
- [GitHub SSH documentation](https://docs.github.com/en/authentication/connecting-to-github-with-ssh)
