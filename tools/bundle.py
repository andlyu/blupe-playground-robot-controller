"""Bundle one prebuilt wheel with its Linux installer; never include local config."""
import hashlib
from pathlib import Path
import tarfile

root=Path(__file__).resolve().parents[1]
version='0.1.0a1'
wheels=list((root/'dist').glob('*.whl'))
if len(wheels)!=1:
    raise SystemExit('Build exactly one wheel in dist/ first: python -m pip wheel --no-deps . -w dist')
name=f'blupe-playground-robot-controller-{version}'
archive=root/'dist'/f'{name}.tar.gz'
files=[root/'install.py',root/'README.md',root/'THIRD-PARTY-NOTICES.md',root/'SOURCE-MANIFEST.json',*list((root/'licenses').glob('*.txt')),wheels[0]]
with tarfile.open(archive,'w:gz') as bundle:
    for file in files:
        bundle.add(file,arcname=f'{name}/{file.relative_to(root)}',recursive=False)
(root/'dist'/'SHA256SUMS').write_text(''.join(f'{hashlib.sha256(file.read_bytes()).hexdigest()}  {file.name}\n' for file in [archive,wheels[0]]))
print(archive)
