<?xml version="1.0" encoding="UTF-8"?>
<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
<!--
  Copyright (c) 2006,2008 Doeke Zanstra
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification, 
  are permitted provided that the following conditions are met:

  Redistributions of source code must retain the above copyright notice, this 
  list of conditions and the following disclaimer. Redistributions in binary 
  form must reproduce the above copyright notice, this list of conditions and the 
  following disclaimer in the documentation and/or other materials provided with 
  the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR 
  OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
  THE POSSIBILITY OF SUCH DAMAGE.
-->

  <xsl:output indent="no" omit-xml-declaration="yes" method="text" encoding="UTF-8" media-type="text/javascript"/>
	<xsl:strip-space elements="*"/>

  <!-- ignore document text -->
  <xsl:template match="text()[preceding-sibling::node() or following-sibling::node()]"/>

  <xsl:template name="attrs">
    <xsl:if test="not(count(attribute::*)=0)">
      <xsl:text>{</xsl:text>
      <xsl:for-each select="attribute::*">
        <xsl:value-of select="local-name()"/>
        <xsl:text>:'</xsl:text>
        <xsl:value-of select="."/>
        <xsl:text>'</xsl:text>
        <xsl:if test="not(position()=last() or last()=1)">
          <xsl:text>,</xsl:text>
        </xsl:if>
      </xsl:for-each>
      <xsl:call-template name="indent"/><xsl:text>}</xsl:text>
    </xsl:if>
  </xsl:template>

  <!-- string -->
  <xsl:template match="text()">
    <xsl:call-template name="escape-string">
      <xsl:with-param name="s" select="."/>
    </xsl:call-template>
  </xsl:template>
  
  <!-- Main template for escaping strings; used by above template and for object-properties 
       Responsibilities: placed quotes around string, and chain up to next filter, escape-bs-string -->
  <xsl:template name="escape-string">
    <xsl:param name="s"/>
    <xsl:text>"</xsl:text>
    <xsl:call-template name="escape-bs-string">
      <xsl:with-param name="s" select="$s"/>
    </xsl:call-template>
    <xsl:text>"</xsl:text>
  </xsl:template>
  
  <!-- Escape the backslash (\) before everything else. -->
  <xsl:template name="escape-bs-string">
    <xsl:param name="s"/>
    <xsl:choose>
      <xsl:when test="contains($s,'\')">
        <xsl:call-template name="escape-quot-string">
          <xsl:with-param name="s" select="concat(substring-before($s,'\'),'\\')"/>
        </xsl:call-template>
        <xsl:call-template name="escape-bs-string">
          <xsl:with-param name="s" select="substring-after($s,'\')"/>
        </xsl:call-template>
      </xsl:when>
      <xsl:otherwise>
        <xsl:call-template name="escape-quot-string">
          <xsl:with-param name="s" select="$s"/>
        </xsl:call-template>
      </xsl:otherwise>
    </xsl:choose>
  </xsl:template>
  
  <!-- Escape the double quote ("). -->
  <xsl:template name="escape-quot-string">
    <xsl:param name="s"/>
    <xsl:choose>
      <xsl:when test="contains($s,'&quot;')">
        <xsl:call-template name="encode-string">
          <xsl:with-param name="s" select="concat(substring-before($s,'&quot;'),'\&quot;')"/>
        </xsl:call-template>
        <xsl:call-template name="escape-quot-string">
          <xsl:with-param name="s" select="substring-after($s,'&quot;')"/>
        </xsl:call-template>
      </xsl:when>
      <xsl:otherwise>
        <xsl:call-template name="encode-string">
          <xsl:with-param name="s" select="$s"/>
        </xsl:call-template>
      </xsl:otherwise>
    </xsl:choose>
  </xsl:template>
  
  <!-- JS: old version of template, doing the same as escape-bs-string and escape-quot-string together, only more complicated -->
  <xsl:template name="escape-bs-and-quot-string-in-one-template">
    <xsl:param name="s"/>
    <!-- First handle  -->
    <xsl:choose>
      <!-- double quote -->
      <xsl:when test="contains($s,'&quot;') and not(contains($s,'\'))">
        <xsl:call-template name="encode-string">
          <xsl:with-param name="s" select="substring-before($s,'&quot;')"/>
        </xsl:call-template>
        <xsl:text>\"</xsl:text>
        <xsl:call-template name="escape-string">
          <xsl:with-param name="s" select="substring-after($s,'&quot;')"/>
        </xsl:call-template>
      </xsl:when>
      <!-- backslash -->
      <xsl:when test="not(contains($s,'&quot;')) and contains($s,'\')">
        <xsl:call-template name="encode-string">
          <xsl:with-param name="s" select="substring-before($s,'\')"/>
        </xsl:call-template>
        <xsl:text>\\</xsl:text>
        <xsl:call-template name="escape-string">
          <xsl:with-param name="s" select="substring-after($s,'\')"/>
        </xsl:call-template>
      </xsl:when>
      <xsl:when test="contains($s,'&quot;') and contains($s,'\')">
        <xsl:choose>
          <!-- double quote before backslash -->
          <xsl:when test="string-length(substring-before($s,'&quot;'))&lt;string-length(substring-before($s,'\'))">
            <xsl:call-template name="encode-string">
              <xsl:with-param name="s" select="substring-before($s,'&quot;')"/>
            </xsl:call-template>
            <xsl:text>\"</xsl:text>
            <xsl:call-template name="escape-string">
              <xsl:with-param name="s" select="substring-after($s,'&quot;')"/>
            </xsl:call-template>
          </xsl:when>
          <!-- backslash before double quote -->
          <xsl:when test="string-length(substring-before($s,'&quot;'))&gt;string-length(substring-before($s,'\'))">
            <xsl:call-template name="encode-string">
              <xsl:with-param name="s" select="substring-before($s,'\')"/>
            </xsl:call-template>
            <xsl:text>\\</xsl:text>
            <xsl:call-template name="escape-string">
              <xsl:with-param name="s" select="substring-after($s,'\')"/>
            </xsl:call-template>
          </xsl:when>
        </xsl:choose>
      </xsl:when>
      <xsl:otherwise>
        <xsl:call-template name="encode-string">
          <xsl:with-param name="s" select="$s"/>
        </xsl:call-template>
      </xsl:otherwise>
    </xsl:choose>
  </xsl:template>

  <!-- Replace tab, line feed and/or carriage return by its matching escape code. Also escape tag-close 
      (</tag> to <\/tag> for client-side javascript compliance). Can't escape backslash
       or double quote here, because they don't replace characters (&#x0; becomes \t), but they prefix 
       characters (\ becomes \\). Besides, backslash should be seperate anyway, because it should be 
       processed first. This function can't do that. -->
  <xsl:template name="encode-string">
    <xsl:param name="s"/>
    <xsl:choose>
      <!-- tab -->
      <xsl:when test="contains($s,'&#x9;')">
        <xsl:call-template name="encode-string">
          <xsl:with-param name="s" select="concat(substring-before($s,'&#x9;'),'\t',substring-after($s,'&#x9;'))"/>
        </xsl:call-template>
      </xsl:when>
      <!-- line feed -->
      <xsl:when test="contains($s,'&#xA;')">
        <xsl:call-template name="encode-string">
          <xsl:with-param name="s" select="concat(substring-before($s,'&#xA;'),'\n',substring-after($s,'&#xA;'))"/>
        </xsl:call-template>
      </xsl:when>
      <!-- carriage return -->
      <xsl:when test="contains($s,'&#xD;')">
        <xsl:call-template name="encode-string">
          <xsl:with-param name="s" select="concat(substring-before($s,'&#xD;'),'\r',substring-after($s,'&#xD;'))"/>
        </xsl:call-template>
      </xsl:when>
      <!-- JS: tag-close -->
      <xsl:when test="contains($s,'&lt;/')">
        <xsl:call-template name="encode-string">
          <xsl:with-param name="s" select="concat(substring-before($s,'&lt;/'),'&lt;\/',substring-after($s,'&lt;/'))"/>
        </xsl:call-template>
      </xsl:when>
      <xsl:otherwise><xsl:value-of select="$s"/></xsl:otherwise>
    </xsl:choose>
  </xsl:template>

  <!-- number (no support for javascript mantissa) -->
  <xsl:template match="text()[string(number())!='NaN']">
    <xsl:value-of select="."/>
  </xsl:template>

  <!-- boolean, case-insensitive -->
  <xsl:template match="text()[translate(.,'TRUE','true')='true']">true</xsl:template>
  <xsl:template match="text()[translate(.,'FALSE','false')='false']">false</xsl:template>

  <!-- JS: Date: YYYY-dd-mm[Thh] -->
  <xsl:template match="text()[string-length()=10 
    and string-length(translate(substring(.,1,4),'0123456789',''))=0 
    and substring(.,5,1)='-' 
    and string-length(translate(substring(.,6,2),'0123456789',''))=0 
    and substring(.,8,1)='-' 
    and string-length(translate(substring(.,9,2),'0123456789',''))=0
    ]">
    <xsl:text>new Date(</xsl:text>
    <xsl:value-of select="substring(.,1,4)"/>
    <xsl:text>,</xsl:text>
    <xsl:value-of select="substring(.,6,2)"/>
    <xsl:text>-1,</xsl:text>
    <xsl:value-of select="substring(.,9,2)"/>
    <xsl:text>)</xsl:text>
  </xsl:template>
  <!-- JS: Date: YYYY-dd-mmThh:mm -->
  <xsl:template match="text()[string-length()=16 
    and string-length(translate(substring(.,1,4),'0123456789',''))=0 
    and substring(.,5,1)='-' 
    and string-length(translate(substring(.,6,2),'0123456789',''))=0 
    and substring(.,8,1)='-' 
    and string-length(translate(substring(.,9,2),'0123456789',''))=0 
    and substring(.,11,1)='T'
    and string-length(translate(substring(.,12,2),'0123456789',''))=0 
    and substring(.,14,1)=':'
    and string-length(translate(substring(.,15,2),'0123456789',''))=0 
    ]">
    <xsl:text>new Date(</xsl:text>
    <xsl:value-of select="substring(.,1,4)"/>
    <xsl:text>,</xsl:text>
    <xsl:value-of select="substring(.,6,2)"/>
    <xsl:text>-1,</xsl:text>
    <xsl:value-of select="substring(.,9,2)"/>
    <xsl:text>,</xsl:text>
    <xsl:value-of select="substring(.,12,2)"/>
    <xsl:text>,</xsl:text>
    <xsl:value-of select="substring(.,15,2)"/>
    <xsl:text>)</xsl:text>
  </xsl:template>
  <!-- JS: Date: YYYY-dd-mmThh:mm:ss -->
  <xsl:template match="text()[string-length()=19 
    and string-length(translate(substring(.,1,4),'0123456789',''))=0 
    and substring(.,5,1)='-' 
    and string-length(translate(substring(.,6,2),'0123456789',''))=0 
    and substring(.,8,1)='-' 
    and string-length(translate(substring(.,9,2),'0123456789',''))=0 
    and substring(.,11,1)='T'
    and string-length(translate(substring(.,12,2),'0123456789',''))=0 
    and substring(.,14,1)=':'
    and string-length(translate(substring(.,15,2),'0123456789',''))=0 
    and substring(.,17,1)=':'
    and string-length(translate(substring(.,18,2),'0123456789',''))=0 
    ]">
    <xsl:text>new Date(</xsl:text>
    <xsl:value-of select="substring(.,1,4)"/>
    <xsl:text>,</xsl:text>
    <xsl:value-of select="substring(.,6,2)"/>
    <xsl:text>-1,</xsl:text>
    <xsl:value-of select="substring(.,9,2)"/>
    <xsl:text>,</xsl:text>
    <xsl:value-of select="substring(.,12,2)"/>
    <xsl:text>,</xsl:text>
    <xsl:value-of select="substring(.,15,2)"/>
    <xsl:text>,</xsl:text>
    <xsl:value-of select="substring(.,18,2)"/>
    <xsl:text>)</xsl:text>
  </xsl:template>

  <!-- object -->
  <xsl:template match="*" name="base">
    <xsl:if test="not(preceding-sibling::*)">{</xsl:if>
    <xsl:call-template name="indent"/>
    <!-- JS: handle attributes -->
    <xsl:call-template name="attrs">
      <xsl:with-param name="attrs" select="./@*"/>
    </xsl:call-template>
    <xsl:call-template name="quote-property">
      <xsl:with-param name="name" select="name()"/>
    </xsl:call-template>
    <xsl:text>:</xsl:text>
    <!-- check type of node -->
    <xsl:choose>
      <!-- null nodes -->
      <xsl:when test="count(child::node())=0">null</xsl:when>
      <!-- other nodes -->
      <xsl:otherwise>
        <xsl:apply-templates select="child::node()"/>
      </xsl:otherwise>
    </xsl:choose>
    <!-- end of type check -->
    <xsl:if test="following-sibling::*">,</xsl:if>
    <xsl:if test="not(following-sibling::*)">}</xsl:if>
  </xsl:template>

  <xsl:template name="quote-property">
    <xsl:param name="name"/>
        <xsl:call-template name="escape-string">
          <xsl:with-param name="s" select="$name"/>
        </xsl:call-template>
  </xsl:template>

  <!-- array -->
  <xsl:template match="PGNInfo//*[count(../*[name(../*)=name(.)])=count(../*) and count(../*)&gt;0]">
    <xsl:if test="not(preceding-sibling::*)">[</xsl:if>
    <xsl:call-template name="indent"/>
    <xsl:choose>
      <xsl:when test="not(child::node())">
    <xsl:call-template name="attrs">
      <xsl:with-param name="attrs" select="./@*"/>
    </xsl:call-template>
      </xsl:when>
      <xsl:otherwise>
        <xsl:apply-templates select="child::node()"/>
      </xsl:otherwise>
    </xsl:choose>
    <xsl:if test="following-sibling::*">,</xsl:if>
    <xsl:if test="not(following-sibling::*)">]</xsl:if>
  </xsl:template>

  <!-- JS: indent for reability -->
  <xsl:template name="indent">
    <xsl:text>
  </xsl:text>
    <xsl:for-each select="ancestor::*">
      <xsl:text>  </xsl:text>
    </xsl:for-each>
  </xsl:template>

  <!-- convert root element to an anonymous container -->
  <xsl:template match="/*">
    <xsl:apply-templates select="node()"/>
  </xsl:template>

  <xsl:template match="LookupEnumerations">
    <xsl:call-template name="indent"/>"LookupEnumerations":[<xsl:apply-templates/>],
  </xsl:template>

  <xsl:template match="LookupBitEnumerations">
    <xsl:call-template name="indent"/>"LookupBitEnumerations":[<xsl:apply-templates/>],
  </xsl:template>

  <xsl:template match="LookupEnumeration">
    <xsl:call-template name="indent"/>{
        "name": "<xsl:value-of select="@Name"/>",
        "maxValue": <xsl:value-of select="@MaxValue"/>,
        "EnumValues": [<xsl:apply-templates/>
        ]
      }<xsl:if test="not(position() = last())">,</xsl:if>
  </xsl:template>

  <xsl:template match="LookupBitEnumeration">
    <xsl:call-template name="indent"/>{
        "name": "<xsl:value-of select="@Name"/>",
        "maxValue": <xsl:value-of select="@MaxValue"/>,
        "EnumBitValues":[<xsl:apply-templates/>
        ]
      }<xsl:if test="not(position() = last())">,</xsl:if>
  </xsl:template>

  <xsl:template match="LookupEnumeration/EnumPair">
    <xsl:call-template name="indent"/>{"Name": "<xsl:value-of select="@Name"/>", "Value": <xsl:value-of select="@Value"/>}<xsl:if test="not(position() = last())">,</xsl:if><xsl:apply-templates/>
  </xsl:template>

  <xsl:template match="LookupBitEnumeration/BitPair">
    <xsl:call-template name="indent"/>{"Name": "<xsl:value-of select="@Name"/>", "Bit": <xsl:value-of select="@Bit"/>}<xsl:if test="not(position() = last())">,</xsl:if><xsl:apply-templates/>
  </xsl:template>

  <xsl:template match="Field/LookupEnumeration">
    <xsl:call-template name="indent"/>"LookupEnumeration": "<xsl:value-of select="."/>"<xsl:text/>
    <xsl:if test="not(following-sibling::*)">}</xsl:if>
  </xsl:template>

  <xsl:template match="Field/LookupBitEnumeration">
    <xsl:call-template name="indent"/>"LookupBitEnumeration": "<xsl:value-of select="."/>"<xsl:text/>
    <xsl:if test="not(following-sibling::*)">}</xsl:if>
  </xsl:template>

  <xsl:template match="EnumValues">
    <xsl:call-template name="indent"/>"EnumValues":[<xsl:apply-templates/>]<xsl:if test="not(following-sibling::*)">}</xsl:if>
  </xsl:template>

  <xsl:template match="EnumBitValues">
    <xsl:call-template name="indent"/>"EnumBitValues":[<xsl:apply-templates/>]<xsl:if test="not(following-sibling::*)">}</xsl:if>
  </xsl:template>

  <xsl:template match="EnumValues/EnumPair">
    <xsl:call-template name="indent"/>{"name": "<xsl:value-of select="@Name"/>", "value": <xsl:value-of select="@Value"/>}<xsl:if test="not(position() = last())">,</xsl:if><xsl:apply-templates/>
  </xsl:template>

  <xsl:template match="EnumBitValues/EnumPair">
    <xsl:call-template name="indent"/>{"<xsl:value-of select="@Bit"/>": "<xsl:value-of select="@Name"/>"}<xsl:if test="not(position() = last())">,</xsl:if><xsl:apply-templates/>
  </xsl:template>

  <xsl:template match="EnumFieldTypeValues">
    <xsl:call-template name="indent"/>"EnumFieldTypeValues":[<xsl:apply-templates/>]<xsl:if test="not(following-sibling::*)">}</xsl:if>
  </xsl:template>

  <xsl:template match="EnumFieldTypeValues/EnumFieldType">
    <xsl:call-template name="indent"/>{"name": "<xsl:value-of select="@Name"/>", "value": <xsl:value-of select="@Value"/>
    <xsl:if test="@Type">, "type": <xsl:value-of select="@Type"/>"</xsl:if>
    <xsl:text>}</xsl:text>
    <xsl:if test="not(position() = last())">,</xsl:if><xsl:apply-templates/>
  </xsl:template>

  <xsl:template match="MissingAttribute">["<xsl:value-of select="node()"/>"]<xsl:if test="not(position() = last())">,</xsl:if>
  </xsl:template>

    <xsl:template match="PGNInfo">
    <xsl:call-template name="indent"/><xsl:apply-templates/><xsl:if test="not(position() = last())">,</xsl:if>
    </xsl:template>

    <xsl:template match="PGNs">
    "PGNs": [
    <xsl:apply-templates/>
    ]
}
    </xsl:template>

</xsl:stylesheet>
