# Post-analysis
Every run gives plots + metrics. Significant runs go into a comparison table.

1. `python post_analysis/scripts/pa.py new <what-changed>`   (prints where to save the CSV)
2. Record the run: `nmpc_node --ros-args -p log_file:=<run>/raw/log.csv`
3. `python post_analysis/scripts/pa.py analyze latest`
4. Fill run.yaml: what_changed, verdict, significant
5. `python post_analysis/scripts/pa.py compare`   → comparison.md

Settings: analysis_config.yaml. Raw data is not in git.
