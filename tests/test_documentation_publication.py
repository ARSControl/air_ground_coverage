"""Deterministic contracts for the mathematical-reference publication assets."""

from pathlib import Path
import unittest
import xml.etree.ElementTree as element_tree


ROOT = Path(__file__).resolve().parents[1]


class MathematicalPublicationTests(unittest.TestCase):
    def test_tex_source_is_a_complete_latex_document(self) -> None:
        source = (ROOT / "docs/latex/mathematical_formulation.tex").read_text()
        self.assertIn("\\begin{document}", source)
        self.assertIn("\\end{document}", source)
        self.assertEqual(source.count("\\begin{align*}"), source.count("\\end{align*}"))
        self.assertEqual(source.count("\\begin{longtable}"), source.count("\\end{longtable}"))
        self.assertNotIn("$$", source)
        self.assertNotIn("\\\\textbf", source)
        self.assertNotIn("\\\\_", source)
        self.assertNotIn("\\[\n\\begin{align*}", source)
        self.assertIn("\\date{}", source)

    def test_pages_workflow_keeps_build_and_deploy_separate(self) -> None:
        workflow = (
            ROOT / ".github/workflows/mathematical-formulation-pdf.yml"
        ).read_text()
        self.assertIn("contents: read", workflow)
        self.assertIn("needs: build", workflow)
        self.assertIn("pages: write", workflow)
        self.assertIn("id-token: write", workflow)
        self.assertIn("uses: actions/upload-pages-artifact@v4", workflow)
        self.assertIn("uses: actions/deploy-pages@v4", workflow)

    def test_publication_diagram_is_valid_svg(self) -> None:
        element_tree.parse(ROOT / "docs/assets/latex_publication_pipeline.svg")


if __name__ == "__main__":
    unittest.main()
