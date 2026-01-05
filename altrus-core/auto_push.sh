#!/bin/bash

# auto_push.sh - Automated Git Push for Altrus Middleware
# Usage: ./auto_push.sh "Your commit message"

set -e  # Exit immediately if a command exits with non-zero status

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMMIT_MSG="${1:-"Update Altrus Middleware"}"

echo "🚀 Starting auto-push for Altrus Middleware..."
echo "📍 Repository: $REPO_ROOT"
echo "📝 Commit message: $COMMIT_MSG"
echo ""

# Check if inside a Git repository
if ! git -C "$REPO_ROOT" rev-parse --git-dir > /dev/null 2>&1; then
    echo "❌ ERROR: Not a Git repository!"
    exit 1
fi

# Check for uncommitted changes
if ! git -C "$REPO_ROOT" diff-index --quiet HEAD --; then
    echo "✅ Uncommitted changes detected. Adding all files..."
    git -C "$REPO_ROOT" add .
else
    echo "ℹ️  No changes to commit."
    exit 0
fi

# Check if there are staged changes to commit
if git -C "$REPO_ROOT" diff --cached --quiet; then
    echo "ℹ️  No staged changes. Nothing to commit."
    exit 0
fi

# Commit changes
echo "💾 Committing changes..."
git -C "$REPO_ROOT" commit -m "$COMMIT_MSG"

# Push to remote
echo "⏫ Pushing to remote repository..."
git -C "$REPO_ROOT" push origin main 2>/dev/null || git -C "$REPO_ROOT" push origin master

echo ""
echo "✅ SUCCESS: Code pushed to Git!"
echo "🔗 Repository: $(git -C "$REPO_ROOT" remote get-url origin)"
echo "🕒 $(date)"