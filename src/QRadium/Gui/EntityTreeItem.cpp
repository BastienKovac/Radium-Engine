/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the examples of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:BSD$
** You may use this file under the terms of the BSD license as follows:
**
** "Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**   * Neither the name of The Qt Company Ltd nor the names of its
**     contributors may be used to endorse or promote products derived
**     from this software without specific prior written permission.
**
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
**
** $QT_END_LICENSE$
**
****************************************************************************/

#include <QRadium/Gui/EntityTreeItem.hpp>

#include <cstdio>

#include <QVector>

namespace QRadium
{

    EntityTreeItem::EntityTreeItem( const QVector<ItemData>& data, EntityTreeItem* parent )
        : m_parentItem( parent )
    {
        m_itemData = data;
    }

    EntityTreeItem::~EntityTreeItem()
    {
        qDeleteAll( m_childItems );
    }

    EntityTreeItem* EntityTreeItem::getChild( int row )
    {
        return m_childItems.value( row );
    }

    uint EntityTreeItem::getChildCount() const
    {
        return m_childItems.count();
    }

    uint EntityTreeItem::getColumnCount() const
    {
        return m_itemData.count();
    }

    EntityTreeItem::ItemData EntityTreeItem::getData( int column ) const
    {
        return m_itemData.value( column );
    }

    EntityTreeItem* EntityTreeItem::getParentItem()
    {
        return m_parentItem;
    }

    uint EntityTreeItem::getRow() const
    {
        if ( m_parentItem )
        {
            return m_parentItem->m_childItems.indexOf( const_cast<EntityTreeItem*>( this ) );
        }
        return 0;
    }

    bool EntityTreeItem::insertChildren( uint position, uint count, uint columns )
    {
        if ( int( position ) > m_childItems.size() )
        {
            return false;
        }

        for ( uint row = 0; row < count; ++row )
        {
            QVector<ItemData> data( columns );
            EntityTreeItem* item = new EntityTreeItem( data, this );
            m_childItems.insert( position, item );
        }

        return true;
    }

    bool EntityTreeItem::insertColumns( uint position, uint columns )
    {
        if ( int( position ) > m_itemData.size() )
        {
            return false;
        }

        for ( uint column = 0; column < columns; ++column )
        {
            m_itemData.insert( position, ItemData() );
        }

        for ( EntityTreeItem* child : m_childItems )
        {
            child->insertColumns( position, columns );
        }

        return true;
    }

    bool EntityTreeItem::removeChildren( uint position, uint count )
    {
        if ( int( position + count ) > m_childItems.size() )
        {
            return false;
        }

        for ( uint row = 0; row < count; ++row )
        {
            delete m_childItems.takeAt( position );
        }

        return true;
    }

    bool EntityTreeItem::removeColumns( uint position, uint columns )
    {
        if ( int( position + columns ) > m_itemData.size() )
        {
            return false;
        }

        for ( uint column = 0; column < columns; ++column )
        {
            m_itemData.remove( position );
        }

        foreach( EntityTreeItem * child, m_childItems )
        {
            child->removeColumns( position, columns );
        }

        return true;
    }

    bool EntityTreeItem::setData( uint column, const ItemData& value )
    {
        if ( int( column ) >= m_itemData.size() )
        {
            return false;
        }

        m_itemData[column] = value;
        return true;
    }

} // namespace Ra
