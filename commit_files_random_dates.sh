#!/usr/bin/env bash

set -e

START_DATE="2025-10-05"
END_DATE=$(date +"%Y-%m-%d")

START_TS=$(date -d "$START_DATE" +%s)
END_TS=$(date -d "$END_DATE" +%s)

if [ "$START_TS" -ge "$END_TS" ]; then
  echo "❌ Start date must be before today"
  exit 1
fi

echo "📅 Commit range: $START_DATE → $END_DATE"
echo "🚀 Starting staged commits..."

# Get tracked, non-deleted files
FILES=$(git ls-files)

for FILE in $FILES; do
  # Skip if file does not exist (safety)
  [ -f "$FILE" ] || continue

  # Random timestamp between start and end
  RAND_TS=$((START_TS + RANDOM % (END_TS - START_TS)))

  COMMIT_DATE=$(date -d "@$RAND_TS" +"%Y-%m-%d %H:%M:%S")

  echo "📦 Committing $FILE @ $COMMIT_DATE"

  git add "$FILE"

  GIT_AUTHOR_DATE="$COMMIT_DATE" \
  GIT_COMMITTER_DATE="$COMMIT_DATE" \
  git commit -m "feat: $FILE"

done

echo "✅ All files committed successfully"
