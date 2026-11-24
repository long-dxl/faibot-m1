# Inherited parts

STEP files taken unchanged from open designs by liyiteng, both under the Apache License 2.0 (see [LICENSE](LICENSE)). The files are byte copies of the ones in `cad/upstream/`; `python hardware/tools/export_parts.py --check-only` compares their SHA-256 sums.

| Folder | Source | Commit |
| --- | --- | --- |
| `am-arm200/` | AM-ARM, https://github.com/liyiteng/AM-ARM (follower arm, `am-arm200/stp/follower`) | `5b03af1ef6c80595bd988e0137cf8e7c35811eb6` |
| `alohamini2/` | AlohaMini, https://github.com/liyiteng/AlohaMini (`AlohaMini2/hardware/mobile_base2/stp`) | `59f024b6005b8516005150b98a3f6daa5d04d851` |

Only the files used in the Faibot M1 assembly are copied. Parts modified by Faibotic are not here; they are listed in [`../../../cad/CHANGES.md`](../../../cad/CHANGES.md) and [`../../../NOTICE`](../../../NOTICE). "AM-ARM" and "AlohaMini" are names of their owners, not Faibotic product names.
