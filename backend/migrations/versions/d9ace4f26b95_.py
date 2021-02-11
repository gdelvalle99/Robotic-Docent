"""empty message

Revision ID: d9ace4f26b95
Revises: 57047e4f6ddb
Create Date: 2021-02-09 22:21:39.976381

"""
from alembic import op
import sqlalchemy as sa
from sqlalchemy.dialects import postgresql

# revision identifiers, used by Alembic.
revision = 'd9ace4f26b95'
down_revision = '57047e4f6ddb'
branch_labels = None
depends_on = None


def upgrade():
    # ### commands auto generated by Alembic - please adjust! ###
    op.drop_table('bizza')
    op.add_column('test', sa.Column('name', sa.String(), nullable=True))
    op.create_unique_constraint(None, 'test', ['id'])
    # ### end Alembic commands ###


def downgrade():
    # ### commands auto generated by Alembic - please adjust! ###
    op.drop_constraint(None, 'test', type_='unique')
    op.drop_column('test', 'name')
    op.create_table('bizza',
    sa.Column('id', postgresql.UUID(), autoincrement=False, nullable=False),
    sa.PrimaryKeyConstraint('id', name='bizza_pkey')
    )
    # ### end Alembic commands ###
