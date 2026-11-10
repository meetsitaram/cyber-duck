"""After cad/fusion/full_build.py has saved a variant: open it from the hub, write its STEP, close it.

  MILLI_RECIPE=feetech bash tools/pipeline.sh step       (in a Fusion turn), or by hand:
  python -I tools/fusion_call.py cad/fusion/export_step.py --write --recipe feetech

fusion_call.py sets RECIPE and points REPO at the checkout it is run from.
Straight after a first save Fusion refuses to reopen it for a while, and a STEP from the document that was just saved has no solids in it.
"""
import os

import adsk.core
import adsk.fusion

RECIPE = 'hybrid'
REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))) if '__file__' in globals() else r'C:\Users\meets\Projects\duck-droids'


def run(_context: str):
    app = adsk.core.Application.get()
    hub = app.data.activeHub
    proj = next(hub.dataProjects.item(i) for i in range(hub.dataProjects.count) if hub.dataProjects.item(i).name == 'my-projects')
    fs = proj.rootFolder.dataFolders
    fd = next(fs.item(i) for i in range(fs.count) if fs.item(i).name == 'duck-droids')
    print('hub', [(fd.dataFiles.item(i).name, fd.dataFiles.item(i).versionNumber) for i in range(fd.dataFiles.count)])
    df = next((fd.dataFiles.item(i) for i in range(fd.dataFiles.count) if fd.dataFiles.item(i).name == f'duck-droids {RECIPE}'), None)
    if df is None:
        raise RuntimeError(f"no 'duck-droids {RECIPE}' in my-projects / duck-droids: run pipeline.sh final first")
    doc = app.documents.open(df, True)
    try:
        d = adsk.fusion.Design.cast(doc.products.itemByProductType('DesignProductType'))
        print('occurrences', d.rootComponent.occurrences.count, 'joints', d.rootComponent.asBuiltJoints.count)
        p = os.path.join(REPO, 'cad', f'duck-droids-{RECIPE}.step')
        em = d.exportManager
        print('step', em.execute(em.createSTEPExportOptions(p)), os.path.getsize(p))
    finally:
        doc.close(False)
    print('open', [app.documents.item(i).name for i in range(app.documents.count)])
