"""The docs cannot drift from the tree: links resolve, the book lists every page,
every package has one."""

import pathlib
import re

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[1]
CONTEXT = ROOT / '.claude' / 'context'
SKIP = {'build', 'install', 'log', 'logs', 'third_party', 'external', '.git', '.pytest_cache'}
LINK = re.compile(r'\]\(([^)\s]+)\)')


def markdown_files():
    for p in sorted(ROOT.rglob('*.md')):
        if not SKIP.intersection(p.relative_to(ROOT).parts):
            yield p


def links(path):
    for target in LINK.findall(path.read_text(encoding='utf-8')):
        if re.match(r'[a-z]+:', target) or target.startswith('#'):
            continue
        yield target.split('#', 1)[0]


@pytest.mark.parametrize('doc', list(markdown_files()), ids=lambda p: str(p.relative_to(ROOT)))
def test_relative_links_resolve(doc):
    broken = [t for t in links(doc) if not (doc.parent / t).exists()]
    assert not broken, f'{doc.relative_to(ROOT)} links to missing {broken}'


def test_the_book_lists_every_context_doc():
    book = CONTEXT / 'README.md'
    listed = {(CONTEXT / t).resolve() for t in links(book)}
    docs = {p.resolve() for p in CONTEXT.rglob('*.md')
            if p != book and 'records' not in p.parts}
    missing = sorted(str(p.relative_to(CONTEXT)) for p in docs - listed)
    assert not missing, f'the book does not list {missing}'


def test_every_package_has_a_page():
    pkgs = {p.parent.name for p in (ROOT / 'src').glob('*/package.xml')}
    pages = {p.stem for p in (CONTEXT / 'packages').glob('*.md')}
    assert pkgs - pages == set(), f'no page for {sorted(pkgs - pages)}'
    assert pages - pkgs == set(), f'page for a package that does not exist: {sorted(pages - pkgs)}'
