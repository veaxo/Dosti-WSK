"""Check that restoration snapshots include live project changes, not build caches."""
from pathlib import Path
import tempfile
import unittest

from project_snapshot import inventory, vault_block


class ProjectSnapshotChecks(unittest.TestCase):
    def test_project_sources_are_included_and_build_caches_are_excluded(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            for name in ["MapperX.java", "field_planner.py", "src/main/java/Robot.java",
                         "tools/mapperx-app/dist/huge.zip", "field_maps/Dosti.json",
                         "field_maps/runs/run_0001.csv", "src/main/deploy/__pycache__/vision.pyc"]:
                target = root / name
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_text("current")
            records = inventory(root)
            self.assertEqual(set(records["files"]), {"MapperX.java", "field_planner.py",
                             "src/main/java/Robot.java", "field_maps/Dosti.json"})
            block = vault_block(root, records)
            self.assertIn("MAPPERX VAULT BEGIN", block)
            self.assertIn('"files": 3', block)

    def test_concurrent_project_edit_is_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            source = root / "MapperX.java"
            source.write_text("first")
            records = inventory(root)
            source.write_text("changed while building")
            with self.assertRaisesRegex(ValueError, "changed during packaging"):
                vault_block(root, records)


if __name__ == "__main__":
    unittest.main()
