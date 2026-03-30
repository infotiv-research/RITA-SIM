import contextlib
import io
import os, re, shutil, time
from pxr import Usd
from omni.kit.scripting import BehaviorScript

class UrdfExporter(BehaviorScript):
    def on_init(self):
        self.sync_urdf_exports()

    @staticmethod
    def _clean_converter_output(text):
        text = re.sub(r"\x1b\[[0-9;]*m", "", text)
        return text.strip()

    def _collect_converter_output(self, stdout_buffer, stderr_buffer):
        return "\n".join(
            part for part in (
                self._clean_converter_output(stdout_buffer.getvalue()),
                self._clean_converter_output(stderr_buffer.getvalue()),
            ) if part
        )

    def _run_converter(self, converter_cls, input_path, output_path):
        stdout_buffer = io.StringIO()
        stderr_buffer = io.StringIO()
        verbose = os.environ.get("ISAAC_URDF_EXPORTER_VERBOSE", "").lower() in {
            "1",
            "true",
            "yes",
            "on",
        }

        try:
            with contextlib.redirect_stdout(stdout_buffer), contextlib.redirect_stderr(stderr_buffer):
                converter_cls.init_from_file(input_path).save_to_file(output_path)
        except Exception as exc:
            output = self._collect_converter_output(stdout_buffer, stderr_buffer)
            if output:
                raise RuntimeError(f"{exc}\n[UsdToUrdf]\n{output}") from exc
            raise

        output = self._collect_converter_output(stdout_buffer, stderr_buffer)

        if verbose and output:
            print(f"[UsdToUrdf] {output}")

    def sync_urdf_exports(self):
        OUT = "/ros2_ws/assets/isaac_urdf_exports"
        SKIP = {"Looks", "PhysicsScene", "thor_table"}
        
        stage = self.stage
        layer = stage.GetRootLayer()
        
        if layer.identifier.startswith("anon:"):
            print("[ERROR] Save the stage to a .usd file first to allow referencing.")
            return
        
        USD_PATH = layer.realPath
        os.makedirs(OUT, exist_ok=True)

        try:
            from nvidia.srl.from_usd.to_urdf import UsdToUrdf as Conv
        except Exception:
            from nvidia.srl.from_usd.to_urdf import UsdToURDF as Conv

        world = stage.GetPrimAtPath("/World")
        if not world or not world.IsValid():
            print("[SKIP] No /World prim found.")
            return

        current_usd_prims = {child.GetName(): child for child in world.GetChildren() if child.GetName() not in SKIP}
        
        existing_folders = {d for d in os.listdir(OUT) if os.path.isdir(os.path.join(OUT, d))}

        removed_count = 0
        for folder_name in existing_folders:
            if folder_name not in current_usd_prims:
                path_to_remove = os.path.join(OUT, folder_name)
                shutil.rmtree(path_to_remove)
                print(f"[REMOVED] {folder_name} folder deleted (prim no longer in stage).")
                removed_count += 1

        exported_count = 0
        for name, child in current_usd_prims.items():
            model_dir = os.path.join(OUT, name)
            out_urdf = os.path.join(model_dir, f"{name}.urdf")
            os.makedirs(model_dir, exist_ok=True)

            run_id = str(int(time.time() * 1000))
            tmp_usd = os.path.join(model_dir, f"__tmp_{name}_{run_id}.usda")

            print(f"[EXPORT] Refreshing {name}")

            s = Usd.Stage.CreateNew(tmp_usd)
            p = s.DefinePrim(f"/{name}", "Xform")
            p.GetReferences().AddReference(USD_PATH, str(child.GetPath()))
            s.GetRootLayer().Save()

            try:
                self._run_converter(Conv, tmp_usd, out_urdf)
                print(f"[OK] Exported {name}")
                exported_count += 1
            except Exception as e:
                print(f"[ERROR] Failed to export {name}: {e}")
            finally:
                if os.path.exists(tmp_usd):
                    os.remove(tmp_usd)

        if exported_count == 0 and removed_count == 0:
            print("[IDLE] No changes detected in /World. URDF library is up to date.")
        else:
            print(f"[COMPLETE] Sync finished. Exported: {exported_count}, Removed: {removed_count}.")
