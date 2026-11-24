"""Verify the DOIs cited in a Markdown document against Crossref.

Stdlib only, network only — NOT part of the test gate. Used to double-check
the reference list of ``docs/SCIENTIFIC_BACKGROUND.md``:

    python3 -m blueboat_gcs.analysis.check_dois blueboat_gcs/docs/SCIENTIFIC_BACKGROUND.md
    python3 -m blueboat_gcs.analysis.check_dois 10.1109/48.219531 10.1007/978-3-540-49886-5

Every DOI found (``10.xxxx/...``) is resolved through ``api.crossref.org``
and its registered title, container, first author and year are printed. A
DOI that does not resolve is reported and the exit code is 1, so the report
cannot silently carry a broken citation. When the Markdown line carrying the
DOI also carries a ``<!-- title: ... -->`` hint, the registered title must
contain that fragment (case-insensitive) or the DOI counts as a mismatch.
"""

from __future__ import annotations

import json
import re
import sys
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path
from typing import Iterable, List, Optional, Tuple

DOI_RE = re.compile(r"10\.\d{4,9}/[^\s\]>,;`'\"]+")
HINT_RE = re.compile(r"<!--\s*title:\s*(.*?)\s*-->")
API = "https://api.crossref.org/works/"
UA = "blueboat-gcs-check-dois/1.0 (mailto:noreply@example.org)"


def _strip(doi: str) -> str:
    """Trim sentence punctuation; drop a trailing ``)`` only when it is
    unbalanced (Elsevier DOIs such as 10.1016/S0146-664X(81)80005-6 contain
    balanced parentheses that belong to the identifier)."""
    doi = doi.rstrip(".,;:")
    while doi.endswith(")") and doi.count(")") > doi.count("("):
        doi = doi[:-1].rstrip(".,;:")
    return doi


def dois_in_text(text: str) -> List[Tuple[str, Optional[str]]]:
    """(doi, title-hint) pairs, in order of first appearance, deduplicated."""
    seen = set()
    out: List[Tuple[str, Optional[str]]] = []
    for line in text.splitlines():
        hint = HINT_RE.search(line)
        for m in DOI_RE.finditer(line):
            d = _strip(m.group(0))
            if d.lower() in seen:
                continue
            seen.add(d.lower())
            out.append((d, hint.group(1) if hint else None))
    return out


def resolve(doi: str, timeout: float = 20.0) -> Optional[dict]:
    url = API + urllib.parse.quote(doi, safe="/")
    req = urllib.request.Request(url, headers={"User-Agent": UA})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            return json.load(resp)["message"]
    except urllib.error.HTTPError as exc:
        if exc.code == 404:
            return None
        raise


def describe(msg: dict) -> str:
    title = (msg.get("title") or [""])[0]
    container = (msg.get("container-title") or [""])[0]
    year = ""
    for key in ("published-print", "published-online", "issued", "created"):
        parts = (msg.get(key) or {}).get("date-parts") or [[None]]
        if parts and parts[0] and parts[0][0]:
            year = str(parts[0][0])
            break
    authors = msg.get("author") or []
    first = ""
    if authors:
        a = authors[0]
        first = a.get("family") or a.get("name") or ""
        if len(authors) > 1:
            first += " et al."
    return f"{first} ({year}) \"{title}\" — {container or msg.get('publisher', '')}"


def check(items: Iterable[Tuple[str, Optional[str]]]) -> int:
    bad = 0
    for doi, hint in items:
        try:
            msg = resolve(doi)
        except Exception as exc:  # network failure: report, do not guess
            print(f"ERROR   {doi}: {exc}")
            bad += 1
            continue
        if msg is None:
            print(f"MISSING {doi}: not registered at Crossref")
            bad += 1
            continue
        desc = describe(msg)
        if hint and hint.lower() not in (msg.get("title") or [""])[0].lower():
            print(f"MISMATCH {doi}: expected title containing {hint!r}\n         got {desc}")
            bad += 1
            continue
        print(f"OK      {doi}: {desc}")
    return bad


def main(argv: List[str]) -> int:
    if not argv:
        print(__doc__)
        return 2
    items: List[Tuple[str, Optional[str]]] = []
    for arg in argv:
        p = Path(arg)
        if p.is_file():
            items.extend(dois_in_text(p.read_text(encoding="utf-8")))
        else:
            items.append((_strip(arg), None))
    bad = check(items)
    print(f"\n{len(items)} DOI(s) checked, {bad} problem(s).")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
