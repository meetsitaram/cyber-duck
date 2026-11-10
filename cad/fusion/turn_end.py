"""The end of a turn at Fusion: save what's saved in the hub and has changed, then close everything, so the next duck starts with nothing open.

tools/pipeline.sh runs this before it gives Fusion up, also after a failed build. Its last line says what's left open, and pipeline.sh puts Fusion on hold if that isn't 0.
A document never saved, such as a build that failed partway, isn't saved: the hub would keep half a model. It's named here, and the next build makes it again.
"""
import adsk.core


def run(_context: str):
    app = adsk.core.Application.get()
    saved, dropped, kept = [], [], []
    for i in range(app.documents.count):
        doc = app.documents.item(i)
        try:
            doc.documentReferences.count
        except RuntimeError:                       # a link its assembly opened: it closes with the assembly, and can't be saved from here
            continue
        try:
            if not doc.isSaved:
                dropped.append(doc.name)
            elif doc.isModified:
                doc.save('End of a Fusion turn')
                saved.append(doc.name)
            else:
                kept.append(doc.name)
        except RuntimeError as e:
            print('could not save', doc.name, str(e)[:160])

    # (bounded, as a link can refuse to close while its assembly is open: an endless loop here froze Fusion for 5 hours on 2026-11-05)
    def refs(x):
        try:
            return x.documentReferences.count
        except RuntimeError:
            return -1

    for _ in range(300):
        n = app.documents.count
        if not n:
            break
        for doc in sorted((app.documents.item(i) for i in range(n)), key=lambda x: -refs(x)):
            try:
                doc.close(False)
            except RuntimeError:
                pass
            adsk.doEvents()
            if app.documents.count < n:
                break
        else:
            print('these would not close:', [app.documents.item(i).name for i in range(n)])
            break
    print('saved', saved, 'already saved', kept, 'never saved, dropped', dropped)
    print('open now', app.documents.count)
