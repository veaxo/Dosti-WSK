"""Focused checks for corruption, unsafe paths, and honest missing-library diagnostics."""
import hashlib
import base64
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

import bundle_bootstrap as bootstrap


class BundleChecks(unittest.TestCase):
    def test_macos_uses_resources_instead_of_framework_symlinks(self):
        with patch.object(sys, "frozen", True, create=True), patch.object(sys, "platform", "darwin"), \
             patch.object(sys, "executable", "/tmp/Shuffleboard.app/Contents/MacOS/Shuffleboard"):
            self.assertEqual(bootstrap.resource_root(), Path("/tmp/Shuffleboard.app/Contents/Resources").resolve())

    def test_corrupted_and_missing_files_are_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            file = root / "main.py"
            file.write_bytes(b"original")
            manifest = {"files": {"main.py": {"size": 8, "sha256": bootstrap.sha256(file)}}}
            self.assertTrue(bootstrap.verify_files(root, manifest)["ok"])
            file.write_bytes(b"modified")
            with self.assertRaisesRegex(ValueError, "main.py"):
                bootstrap.verify_files(root, manifest)
            file.unlink()
            with self.assertRaises(ValueError):
                bootstrap.verify_files(root, manifest)

    def test_path_traversal_is_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            manifest = {"files": {"../outside": {"size": 0, "sha256": "0" * 64}}}
            with self.assertRaisesRegex(ValueError, "путь"):
                bootstrap.verify_files(folder, manifest)

    def test_modified_manifest_cannot_replace_the_pinned_digest(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "bundle-manifest.json"
            path.write_text(json.dumps({"schema": 1, "files": {}}))
            pin = bootstrap.sha256(path)
            with patch.dict(sys.modules, {"bundle_info": types.SimpleNamespace(MANIFEST_SHA256=pin)}):
                self.assertEqual(bootstrap.read_manifest(folder)["schema"], 1)
                path.write_text(path.read_text() + " ")
                with self.assertRaisesRegex(ValueError, "манифест"):
                    bootstrap.read_manifest(folder)

    def test_failed_native_import_is_reported(self):
        original = bootstrap.importlib.import_module

        def import_module(name):
            if name == "cv2":
                raise ImportError("missing native DLL")
            return original(name)

        with patch.object(bootstrap.importlib, "import_module", side_effect=import_module):
            failed = [r for r in bootstrap.dependency_check() if r["module"] == "cv2"]
        self.assertEqual(len(failed), 1)
        self.assertFalse(failed[0]["ok"])
        self.assertIn("missing native DLL", failed[0]["error"])

    def test_embedded_installer_is_verified_after_extraction(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            payload = root / "bundle_payload/offline/MapperX_offline.py"
            payload.parent.mkdir(parents=True)
            name = "WebView2-x64.exe"
            content = b"MZ offline installer bytes"
            manifest = {"prerequisites": {name: {"size": len(content), "sha256": hashlib.sha256(content).hexdigest()}}}
            block = ("# ==== SHUFFLEBOARD PREREQUISITE " + name + " BEGIN ====\n# "
                     + base64.b64encode(content).decode() + "\n# ==== SHUFFLEBOARD PREREQUISITE " + name + " END ====\n")
            payload.write_text(block)
            target = bootstrap.extract_prerequisite(name, root, manifest, root / "cache")
            self.assertEqual(target.read_bytes(), content)
            target.unlink()
            payload.write_text(block.replace(base64.b64encode(content).decode(), base64.b64encode(b"wrong bytes").decode()))
            with self.assertRaisesRegex(ValueError, "установщик"):
                bootstrap.extract_prerequisite(name, root, manifest, root / "cache")

    def test_offline_verifier_allows_code_updates_but_rejects_changed_payload(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            begin, end = "# PAYLOAD BEGIN", "# PAYLOAD END"
            content = b"immutable library bytes"
            block = begin + "\n# " + base64.b64encode(content).decode() + "\n" + end + "\n"
            main = root / "main.py"
            main.write_text("# original code\n" + block)
            records = [{"begin": begin, "end": end, "size": len(content), "sha256": hashlib.sha256(content).hexdigest()}]
            template = Path(__file__).with_name("offline_verify.py").read_text()
            template = template.replace("@@PAYLOAD_RECORDS@@", json.dumps(records)).replace("@@MAIN_SHA256@@", bootstrap.sha256(main))
            helper = root / "verify.py"
            helper.write_text(template)
            spec = importlib.util.spec_from_file_location("offline_verifier_test", helper)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            module.verify(main)
            main.write_text("# legitimate new code and vault\n" + block)
            with patch.object(module.hashlib, "file_digest", None):
                module.verify(main)
            bad = base64.b64encode(b"corrupted library bytes").decode()
            main.write_text("# new code\n" + block.replace(base64.b64encode(content).decode(), bad))
            with self.assertRaisesRegex(ValueError, "Damaged"):
                module.verify(main)


if __name__ == "__main__":
    unittest.main()
