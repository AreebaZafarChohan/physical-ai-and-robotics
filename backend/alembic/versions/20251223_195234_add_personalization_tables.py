"""Add personalization tables

Revision ID: 20251223_195234_add_personalization_tables
Revises: c67cd89d8505
Create Date: 2025-12-23 19:52:34.000000

"""
from typing import Sequence, Union

from alembic import op
import sqlalchemy as sa
import sqlmodel
from sqlalchemy.dialects import postgresql

# revision identifiers, used by Alembic.
revision: str = '20251223_195234_add_personalization_tables'
down_revision: Union[str, Sequence[str], None] = 'c67cd89d8505'
branch_labels: Union[str, Sequence[str], None] = None
depends_on: Union[str, Sequence[str], None] = None


def upgrade() -> None:
    """Upgrade schema."""
    # Create user_profiles table
    op.create_table('user_profiles',
        sa.Column('id', sa.Integer(), nullable=False),
        sa.Column('user_id', sa.Integer(), nullable=True),
        sa.Column('software_background', postgresql.JSON(astext_type=sa.Text()), nullable=False),
        sa.Column('hardware_background', postgresql.JSON(astext_type=sa.Text()), nullable=False),
        sa.Column('experience_level', sa.VARCHAR(length=20), nullable=False),
        sa.Column('created_at', sa.DateTime(), nullable=False),
        sa.Column('updated_at', sa.DateTime(), nullable=False),
        sa.ForeignKeyConstraint(['user_id'], ['user.id'], ),
        sa.PrimaryKeyConstraint('id'),
        sa.UniqueConstraint('user_id')
    )
    op.create_index(op.f('ix_user_profiles_user_id'), 'user_profiles', ['user_id'], unique=True)

    # Create personalization_rules table
    op.create_table('personalization_rules',
        sa.Column('id', sa.Integer(), nullable=False),
        sa.Column('content_path', sa.String(), nullable=False),
        sa.Column('user_profile_criteria', postgresql.JSONB(astext_type=sa.Text()), nullable=False),
        sa.Column('content_variants', postgresql.JSONB(astext_type=sa.Text()), nullable=False),
        sa.Column('description', sa.Text(), nullable=True),
        sa.Column('priority', sa.Integer(), nullable=False),
        sa.Column('is_active', sa.Boolean(), nullable=False),
        sa.Column('created_at', sa.DateTime(), nullable=False),
        sa.Column('updated_at', sa.DateTime(), nullable=False),
        sa.PrimaryKeyConstraint('id')
    )
    op.create_index(op.f('ix_personalization_rules_content_path'), 'personalization_rules', ['content_path'], unique=False)

    # Create personalization_cache table
    op.create_table('personalization_cache',
        sa.Column('id', sa.Integer(), nullable=False),
        sa.Column('user_profile_hash', sa.String(), nullable=False),
        sa.Column('content_path', sa.String(), nullable=False),
        sa.Column('personalized_content', sa.Text(), nullable=False),
        sa.Column('cache_created_at', sa.DateTime(), nullable=False),
        sa.Column('cache_expires_at', sa.DateTime(), nullable=False),
        sa.PrimaryKeyConstraint('id')
    )
    op.create_index(op.f('ix_personalization_cache_content_path'), 'personalization_cache', ['content_path'], unique=False)
    op.create_index(op.f('ix_personalization_cache_user_profile_hash'), 'personalization_cache', ['user_profile_hash'], unique=False)

    # Create content_markers table
    op.create_table('content_markers',
        sa.Column('id', sa.Integer(), nullable=False),
        sa.Column('content_path', sa.String(), nullable=False),
        sa.Column('marker_type', sa.String(length=100), nullable=False),
        sa.Column('marker_location', sa.Text(), nullable=True),
        sa.Column('description', sa.Text(), nullable=True),
        sa.Column('created_at', sa.DateTime(), nullable=False),
        sa.PrimaryKeyConstraint('id')
    )
    op.create_index(op.f('ix_content_markers_content_path'), 'content_markers', ['content_path'], unique=False)


def downgrade() -> None:
    """Downgrade schema."""
    # Drop content_markers table
    op.drop_index(op.f('ix_content_markers_content_path'), table_name='content_markers')
    op.drop_table('content_markers')

    # Drop personalization_cache table
    op.drop_index(op.f('ix_personalization_cache_user_profile_hash'), table_name='personalization_cache')
    op.drop_index(op.f('ix_personalization_cache_content_path'), table_name='personalization_cache')
    op.drop_table('personalization_cache')

    # Drop personalization_rules table
    op.drop_index(op.f('ix_personalization_rules_content_path'), table_name='personalization_rules')
    op.drop_table('personalization_rules')

    # Drop user_profiles table
    op.drop_index(op.f('ix_user_profiles_user_id'), table_name='user_profiles')
    op.drop_table('user_profiles')
