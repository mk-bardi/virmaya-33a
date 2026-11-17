# Results

Saved test results worth keeping: dashboard reports, log extracts, sim versus real comparisons.

Git ignores this folder by default so day-to-day runs do not fill the repo. Add a result on purpose:

```bash
git add -f results/2026-11-20_hover_sim_vs_real/
```

One folder per test, named `<date>_<test>`, with a short `notes.md` saying the commit of the model, the world, what was flown and what was learned.
