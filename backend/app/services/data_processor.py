import markdown_it
from typing import List, Dict, Tuple
import frontmatter
import os # Added os for path manipulation

md = markdown_it.MarkdownIt()

def _extract_text_from_markdown_internal(markdown_content: str) -> str:
    """
    Extracts plain text from Markdown content using markdown-it-py's parsing capabilities.
    """
    tokens = md.parse(markdown_content)
    plain_text = []
    for token in tokens:
        if token.type == 'text':
            plain_text.append(token.content)
        elif token.type == 'code_inline' or token.type == 'code_block':
            plain_text.append(token.content)
        elif token.type == 'html_inline' or token.type == 'html_block':
            pass
        elif token.children:
            for child in token.children:
                if child.type == 'text':
                    plain_text.append(child.content)
    return " ".join(plain_text).strip()

def extract_text_from_document(file_path: str, content: str) -> str:
    """
    Extracts plain text from various document types based on file extension.
    """
    file_extension = os.path.splitext(file_path)[1].lower()

    if file_extension == '.md' or file_extension == '.mdx':
        return _extract_text_from_markdown_internal(content)
    elif file_extension == '.pdf':
        # Placeholder for PDF text extraction logic (e.g., using PyPDF2 or pdfminer.six)
        raise NotImplementedError("PDF text extraction is not yet implemented.")
    elif file_extension == '.txt':
        return content
    else:
        print(f"Warning: Unsupported file extension {file_path} with extension {file_extension}. Using content as is.")
        return content

def chunk_text(text: str, chunk_size: int = 500, chunk_overlap: int = 50) -> List[str]:
    """
    Splits text into smaller chunks with optional overlap.
    A more advanced chunking strategy (e.g., based on sentences, paragraphs) would be better.
    """
    chunks = []
    if not text:
        return chunks

    words = text.split()
    if not words:
        return chunks

    current_chunk = []
    for word in words:
        current_chunk.append(word)
        if len(" ".join(current_chunk)) >= chunk_size:
            chunks.append(" ".join(current_chunk))
            overlap_words_count = max(0, min(len(current_chunk), int(chunk_overlap / (len(word) + 1))))
            current_chunk = current_chunk[-overlap_words_count:]
            if not current_chunk:
                current_chunk = []
    if current_chunk:
        chunks.append(" ".join(current_chunk))

    return chunks

def extract_metadata_from_markdown(file_path: str, markdown_content: str) -> Dict:
    """
    Extracts metadata (chapter, source_link, paragraph_id) from a Docusaurus Markdown file.
    """
    metadata = {
        "chapter": "Unknown Chapter",
        "source_link": file_path,
        "paragraph_id": "auto-generated-id" # Needs a more robust generation logic
    }

    try:
        post = frontmatter.loads(markdown_content)
        if 'title' in post.metadata:
            metadata['chapter'] = post.metadata['title']
    except Exception:
        pass

    parts = file_path.split('/')
    if len(parts) >= 3 and parts[-3] == 'docs' and parts[-2].startswith('0'):
        module_name_part = parts[-2]
        cleaned_chapter = " ".join(module_name_part.split('-')[1:]).title()
        metadata['chapter'] = cleaned_chapter

    return metadata