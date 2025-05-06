<xsl:stylesheet xmlns:xsl="http://www.w3.org/1999/XSL/Transform" xmlns:xs="http://www.w3.org/2001/XMLSchema" xmlns="http://www.w3.org/1999/xhtml" version="1.0" exclude-result-prefixes="xs">
  <xsl:output method="text" />

  <xsl:template name="pgn-list">
    <xsl:for-each select="/PGNDefinitions/PGNs/*">
      <xsl:choose>
        <xsl:when test="./Fallback = 'true'">
        </xsl:when>
        <xsl:otherwise>
          <xsl:for-each select="Fields/*">
            <xsl:value-of select="../../PGN"/>
            <xsl:text>,</xsl:text>
            <xsl:value-of select="../../Type"/>
            <xsl:text>,</xsl:text>
            <xsl:value-of select="../../Length"/>
            <xsl:value-of select="../../MinLength"/>
            <xsl:text>,</xsl:text>
            <xsl:value-of select="../../Priority"/>
            <xsl:text>,</xsl:text>
            <xsl:value-of select="../../FieldCount"/>
            <xsl:text>,</xsl:text>
            <xsl:value-of select="Order"/>
            <xsl:text>,"</xsl:text>
            <xsl:value-of select="Name"/>
            <xsl:text>",</xsl:text>
            <xsl:value-of select="FieldType"/>
            <xsl:text>,</xsl:text>
            <xsl:value-of select="Signed"/>
            <xsl:text>,</xsl:text>
            <xsl:value-of select="BitLength"/>
            <xsl:text>,</xsl:text>
            <xsl:value-of select="Unit"/>
            <xsl:text>,</xsl:text>
            <xsl:value-of select="Resolution"/>
            <xsl:text>,</xsl:text>
            <xsl:value-of select="Offset"/>
            <xsl:text>,</xsl:text>
            <xsl:value-of select="RangeMin"/>
            <xsl:text>,</xsl:text>
            <xsl:value-of select="RangeMax"/>
            <xsl:text>
</xsl:text>
          </xsl:for-each>
        </xsl:otherwise>
      </xsl:choose>
    </xsl:for-each>
  </xsl:template>
  <xsl:template match="/">
    <xsl:call-template name="pgn-list"/>
  </xsl:template>
</xsl:stylesheet>
