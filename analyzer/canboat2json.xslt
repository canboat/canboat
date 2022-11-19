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
      <xsl:call-template name="outer-indent"/><xsl:text>{</xsl:text>
      <xsl:for-each select="attribute::*">
        <xsl:call-template name="indent"/>
        <xsl:text>"</xsl:text>
        <xsl:value-of select="local-name()"/>
        <xsl:text>":"</xsl:text>
        <xsl:value-of select="."/>
        <xsl:text>"</xsl:text>
        <xsl:if test="not(position()=last() or last()=1)">
          <xsl:text>,</xsl:text>
        </xsl:if>
      </xsl:for-each>
      <xsl:call-template name="outer-indent"/><xsl:text>}</xsl:text>
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

  <!-- object -->
  <xsl:template match="*" name="base">
    <xsl:if test="not(preceding-sibling::*)">
      <xsl:call-template name="outer-indent"/><xsl:text>{</xsl:text>
    </xsl:if>
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
    <xsl:if test="following-sibling::*"><xsl:text>,</xsl:text></xsl:if>
    <xsl:if test="not(following-sibling::*)">
      <xsl:call-template name="outer-indent"/><xsl:text>}</xsl:text>
    </xsl:if>
  </xsl:template>

  <xsl:template name="attrs2">
    <xsl:if test="not(count(attribute::*)=0)">
      <xsl:for-each select="attribute::*">
        <xsl:call-template name="indent"/>
        <xsl:text>"</xsl:text>
        <xsl:value-of select="local-name()"/>
        <xsl:text>":"</xsl:text>
        <xsl:value-of select="."/>
        <xsl:text>",</xsl:text>
      </xsl:for-each>
    </xsl:if>
  </xsl:template>

  <!-- object with attrs in object -->
  <xsl:template name="base2">
    <xsl:call-template name="outer-indent"/><xsl:text>{</xsl:text>
    <xsl:call-template name="attrs2">
      <xsl:with-param name="attrs2" select="./@*"/>
    </xsl:call-template>
    <xsl:for-each select="child::node()">
      <xsl:call-template name="outer-indent"/>
      <xsl:text>"</xsl:text>
      <xsl:value-of select="local-name()"/>
      <xsl:text>":</xsl:text>
      <xsl:apply-templates select="child::node()"/>
      <xsl:if test="following-sibling::*">,</xsl:if>
    </xsl:for-each>
    <xsl:call-template name="indent"/><xsl:text>}</xsl:text>
    <xsl:if test="following-sibling::*">,</xsl:if>
  </xsl:template>

  <xsl:template name="quote-property">
    <xsl:param name="name"/>
    <xsl:call-template name="indent"/>
    <xsl:call-template name="escape-string">
      <xsl:with-param name="s" select="$name"/>
    </xsl:call-template>
  </xsl:template>

  <!-- array -->
  <xsl:template match="PGNInfo//*[count(../*[name(../*)=name(.)])=count(../*) and count(../*)&gt;0]">
    <xsl:if test="not(preceding-sibling::*)">[</xsl:if>
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
    <xsl:if test="not(following-sibling::*)">
      <xsl:call-template name="outer-indent"/>]</xsl:if>
  </xsl:template>

  <xsl:template name="indent">
    <xsl:text>
  </xsl:text>
    <xsl:for-each select="ancestor::*">
      <xsl:text>  </xsl:text>
    </xsl:for-each>
  </xsl:template>

  <xsl:template name="outer-indent">
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

  <xsl:template match="MissingEnumerations">
    <xsl:call-template name="indent"/><xsl:text>"MissingEnumerations":[</xsl:text>
    <xsl:apply-templates/>
    <xsl:call-template name="indent"/><xsl:text>],</xsl:text>
  </xsl:template>

  <xsl:template match="PhysicalQuantities">
    <xsl:call-template name="indent"/><xsl:text>"PhysicalQuantities":[</xsl:text>
    <xsl:apply-templates/>
    <xsl:call-template name="indent"/><xsl:text>],</xsl:text>
  </xsl:template>

  <xsl:template match="FieldTypes">
    <xsl:call-template name="indent"/><xsl:text>"FieldTypes":[</xsl:text>
    <xsl:apply-templates/>
    <xsl:call-template name="indent"/><xsl:text>],</xsl:text>
  </xsl:template>

  <xsl:template match="LookupEnumerations">
    <xsl:call-template name="indent"/><xsl:text>"LookupEnumerations":[</xsl:text>
    <xsl:apply-templates/>
    <xsl:call-template name="indent"/><xsl:text>],</xsl:text>
  </xsl:template>

  <xsl:template match="LookupIndirectEnumerations">
    <xsl:call-template name="indent"/><xsl:text>"LookupIndirectEnumerations":[</xsl:text>
    <xsl:apply-templates/>
    <xsl:call-template name="indent"/><xsl:text>],</xsl:text>
  </xsl:template>

  <xsl:template match="LookupBitEnumerations">
    <xsl:call-template name="indent"/><xsl:text>"LookupBitEnumerations":[</xsl:text>
    <xsl:apply-templates/>
    <xsl:call-template name="indent"/><xsl:text>],</xsl:text>
  </xsl:template>

  <xsl:template match="MissingEnumerations/MissingAttribute">
    <xsl:call-template name="indent"/><xsl:text>{</xsl:text>
    <xsl:call-template name="indent"/><xsl:text>  "Name":"</xsl:text>
    <xsl:value-of select="@Name"/><xsl:text>",</xsl:text>
    <xsl:call-template name="indent"/><xsl:text>  "Value":"</xsl:text>
    <xsl:value-of select="."/><xsl:text>"</xsl:text>
    <xsl:call-template name="indent"/><xsl:text>}</xsl:text>
    <xsl:if test="not(position() = last())">,</xsl:if>
  </xsl:template>

  <xsl:template match="PhysicalQuantities/PhysicalQuantity">
    <xsl:call-template name="base2"/>
  </xsl:template>

  <xsl:template match="FieldTypes/FieldType">
    <xsl:call-template name="base2"/>
  </xsl:template>

  <xsl:template match="LookupEnumerations/LookupEnumeration">
    <xsl:call-template name="indent"/>{
        "Name":"<xsl:value-of select="@Name"/>",
        "MaxValue":<xsl:value-of select="@MaxValue"/>,
        "EnumValues":[<xsl:apply-templates/>
        ]
      }<xsl:if test="not(position() = last())">,</xsl:if>
  </xsl:template>

  <xsl:template match="LookupIndirectEnumerations/LookupIndirectEnumeration">
    <xsl:call-template name="indent"/>{
        "Name":"<xsl:value-of select="@Name"/>",
        "MaxValue":<xsl:value-of select="@MaxValue"/>,
        "EnumValues":[<xsl:apply-templates/>
        ]
      }<xsl:if test="not(position() = last())">,</xsl:if>
  </xsl:template>

  <xsl:template match="LookupBitEnumerations/LookupBitEnumeration">
    <xsl:call-template name="indent"/>{
        "Name":"<xsl:value-of select="@Name"/>",
        "MaxValue":<xsl:value-of select="@MaxValue"/>,
        "EnumBitValues":[<xsl:apply-templates/>
        ]
      }<xsl:if test="not(position() = last())">,</xsl:if>
  </xsl:template>

  <xsl:template match="LookupEnumeration/EnumPair">
    <xsl:call-template name="indent"/>  {"Name":"<xsl:value-of select="@Name"/>", "Value":<xsl:value-of select="@Value"/>}<xsl:if test="not(position() = last())">,</xsl:if><xsl:apply-templates/>
  </xsl:template>

  <xsl:template match="LookupIndirectEnumeration/EnumTriplet">
    <xsl:call-template name="indent"/>  {"Name":"<xsl:value-of select="@Name"/>", "Value1":<xsl:value-of select="@Value1"/>, "Value2":<xsl:value-of select="@Value2"/>}<xsl:if test="not(position() = last())">,</xsl:if><xsl:apply-templates/>
  </xsl:template>

  <xsl:template match="LookupBitEnumeration/BitPair">
    <xsl:call-template name="indent"/>  {"Name":"<xsl:value-of select="@Name"/>", "Bit":<xsl:value-of select="@Bit"/>}<xsl:if test="not(position() = last())">,</xsl:if><xsl:apply-templates/>
  </xsl:template>

  <xsl:template match="EnumValues">
    <xsl:call-template name="indent"/>"EnumValues":[<xsl:apply-templates/>]<xsl:if test="not(following-sibling::*)">}</xsl:if>
  </xsl:template>

  <xsl:template match="EnumBitValues">
    <xsl:call-template name="indent"/>"EnumBitValues":[<xsl:apply-templates/>]<xsl:if test="not(following-sibling::*)">}</xsl:if>
  </xsl:template>

  <xsl:template match="EnumValues/EnumPair">
    <xsl:call-template name="indent"/>  {"name":"<xsl:value-of select="@Name"/>", "value":<xsl:value-of select="@Value"/>}<xsl:if test="not(position() = last())">,</xsl:if><xsl:apply-templates/>
  </xsl:template>

  <xsl:template match="EnumBitValues/EnumPair">
    <xsl:call-template name="indent"/>  {"<xsl:value-of select="@Bit"/>":"<xsl:value-of select="@Name"/>"}<xsl:if test="not(position() = last())">,</xsl:if><xsl:apply-templates/>
  </xsl:template>


  <xsl:template match="PGNInfo">
    <xsl:call-template name="indent"/><xsl:apply-templates/><xsl:if test="not(position() = last())">,</xsl:if>
  </xsl:template>

  <xsl:template match="PGNs">
    "PGNs":[
      <xsl:for-each select="*">
        <xsl:choose>
          <xsl:when test="./Fallback = 'true'">
          </xsl:when>
          <xsl:otherwise>
            <xsl:apply-templates/>
            <xsl:if test="following-sibling::*">,</xsl:if>
          </xsl:otherwise>
        </xsl:choose>
      </xsl:for-each>
    ]
  }</xsl:template>

</xsl:stylesheet>
