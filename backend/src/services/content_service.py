import os
from pathlib import Path
from typing import Optional
from fastapi import HTTPException, status


class ContentService:
    """
    Service class for content-related operations
    """
    
    def __init__(self, content_base_path: str = "content"):
        """
        Initialize the content service with the base path for content files
        """
        self.content_base_path = Path(content_base_path)
        
        # Verify the content directory exists
        if not self.content_base_path.exists():
            raise ValueError(f"Content directory does not exist: {self.content_base_path}")
    
    async def get_content_by_path(self, content_path: str) -> str:
        """
        Get content by its path in the content directory structure
        """
        # Sanitize the path to prevent directory traversal attacks
        # Remove leading slash and normalize the path
        normalized_path = os.path.normpath(content_path.lstrip('/'))

        # Candidate paths to support Docusaurus ordering prefixes (e.g. 01-*) and
        # content stored as either <slug>.md(x) or <dir>/index.md(x).
        candidates: list[Path] = []

        base = self.content_base_path
        rel = Path(normalized_path)

        def _add_candidates(p: Path) -> None:
            if p.suffix:
                candidates.append(p)
            else:
                candidates.extend([
                    p.with_suffix('.md'),
                    p.with_suffix('.mdx'),
                    p / 'index.md',
                    p / 'index.mdx',
                ])

        _add_candidates(base / rel)

        # If the path looks like Docusaurus docId (numeric prefixes stripped), try to
        # match on disk by ignoring leading NN- ordering prefixes.
        if len(rel.parts) >= 2:
            module_slug = rel.parts[0]
            file_slug = rel.parts[-1]
            # Keep intermediate parts (e.g. quizzes/<file>, try-with-ai/<file>)
            sub_path = Path(*rel.parts[1:])
            parent_subdir = Path(*rel.parts[1:-1]) if len(rel.parts) > 2 else Path()

            for module_dir in base.glob(f"*-{module_slug}"):
                if not module_dir.is_dir():
                    continue

                # Candidates preserving subdirectories (e.g. 01-module-ros/quizzes/quiz-foo.md)
                _add_candidates(module_dir / sub_path)

                # Also try numeric-prefixed filenames within the expected parent subdir
                target_dir = (module_dir / parent_subdir) if str(parent_subdir) else module_dir
                if target_dir.is_dir():
                    for ext in ('.md', '.mdx'):
                        for prefixed_file in target_dir.glob(f"*-{file_slug}{ext}"):
                            candidates.append(prefixed_file)

        # Verify and return first readable match
        for full_path in candidates:
            # Verify that the path is within the content directory (security check)
            try:
                full_path.resolve().relative_to(self.content_base_path.resolve())
            except ValueError:
                continue

            if full_path.suffix.lower() and full_path.suffix.lower() not in ['.md', '.mdx', '.txt', '.html']:
                continue

            if full_path.is_file():
                try:
                    with open(full_path, 'r', encoding='utf-8') as file:
                        return file.read()
                except Exception as e:
                    raise HTTPException(
                        status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                        detail=f"Error reading content: {str(e)}"
                    )

        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Content not found: {content_path}"
        )

    async def get_content_markers(self, content: str) -> list:
        """
        Extract personalization markers from content
        """
        # Looking for markers in the format: {{personalize:marker_type}}...{{personalize:end}}
        import re
        
        # Regular expression to match personalization markers
        pattern = r'\{\{personalize:([^}]+)\}\}([\s\S]*?)\{\{personalize:end\}\}'
        matches = re.findall(pattern, content)
        
        markers = []
        for marker_type, marker_content in matches:
            markers.append({
                'type': marker_type,
                'content': marker_content.strip()
            })
        
        return markers
    
    async def get_all_content_paths(self, extension: str = ".md") -> list:
        """
        Get all content paths with a specific extension
        """
        paths = []
        for file_path in self.content_base_path.rglob(f'*{extension}'):
            # Convert to relative path from content base
            relative_path = file_path.relative_to(self.content_base_path)
            paths.append(str(relative_path))
        
        return paths