# Corpus backup and Hugging Face upload protocol

Every recording session is backed up off-machine before the next collection block. Keep
the local dataset root intact until the remote revision is verified. Timing sidecars under
`meta/alignment/`, camera/session manifests, annotations, and the dataset card are part of
the artifact—not optional lab notes.

## One-time authentication check

```bash
hf auth whoami
```

Use the modern `hf` CLI, not the deprecated `huggingface-cli`. Authenticate with the
configured credential store or `HF_TOKEN`; never put a token in a command copied into a
session report.

## Session upload

Create the private dataset repository once if recording did not already create it:

```bash
hf repos create ${HF_USER}/telemetry-failure-corpus --type dataset --private --exist-ok
```

After the local alignment audit passes, upload the complete dataset root in one commit:

```bash
hf upload ${HF_USER}/telemetry-failure-corpus /path/to/local/dataset . \
    --type dataset --private \
    --commit-message 'session YYYY-MM-DD_a: N episodes, alignment PASS'
```

Do not exclude `meta/alignment/`. Do not use `--delete` during collection. A bad local glob
must not remove prior remote evidence.

## Verification

```bash
hf datasets list ${HF_USER}/telemetry-failure-corpus --tree --recursive --human-readable
```

Confirm the new parquet/video chunks, episode metadata, and every expected
`meta/alignment/episode_NNNNNN.jsonl` are present. Record the Hub commit/revision in the
session log and annotation metadata. If verification fails, stop collection and preserve
both the local root and CLI output until resolved.

The repository remains private during collection. Public release happens only after
privacy review, annotation validation, dataset-card completion, and reproduction of the
headline metrics from a pinned revision.

