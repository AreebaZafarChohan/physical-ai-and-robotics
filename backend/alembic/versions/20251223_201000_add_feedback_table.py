"""Add personalization feedback table

Revision ID: 20251223_201000_add_feedback_table
Revises: 20251223_195234_add_personalization_tables
Create Date: 2025-12-23 20:10:00.000000

"""
from typing import Sequence, Union

from alembic import op
import sqlalchemy as sa
import sqlmodel
from sqlalchemy.dialects import postgresql

# revision identifiers, used by Alembic.
revision: str = '20251223_201000_add_feedback_table'
down_revision: Union[str, Sequence[str], None] = '20251223_195234_add_personalization_tables'
branch_labels: Union[str, Sequence[str], None] = None
depends_on: Union[str, Sequence[str], None] = None


def upgrade() -> None:
    """Upgrade schema."""
    # Create personalization_feedback table
    op.create_table('personalization_feedback',
        sa.Column('id', sa.Integer(), nullable=False),
        sa.Column('user_id', sa.Integer(), nullable=False),
        sa.Column('chapter_path', sa.String(length=500), nullable=False),
        sa.Column('feedback_score', sa.Integer(), nullable=False),
        sa.Column('feedback_comment', sa.Text(), nullable=True),
        sa.Column('created_at', sa.DateTime(), nullable=False),
        sa.ForeignKeyConstraint(['user_id'], ['user.id'], ),
        sa.CheckConstraint('feedback_score >= 1 AND feedback_score <= 5', name='feedback_score_check'),
        sa.PrimaryKeyConstraint('id')
    )
    op.create_index(op.f('ix_personalization_feedback_chapter_path'), 'personalization_feedback', ['chapter_path'], unique=False)
    op.create_index(op.f('ix_personalization_feedback_user_id'), 'personalization_feedback', ['user_id'], unique=False)


def downgrade() -> None:
    """Downgrade schema."""
    # Drop personalization_feedback table
    op.drop_index(op.f('ix_personalization_feedback_user_id'), table_name='personalization_feedback')
    op.drop_index(op.f('ix_personalization_feedback_chapter_path'), table_name='personalization_feedback')
    op.drop_table('personalization_feedback')
